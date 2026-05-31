"""Process-inspection port + adapters.

The port hides ``psutil`` so trackers stay testable in environments where the
library is absent or the target process is not actually running.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Protocol

try:
    import psutil  # type: ignore
except ImportError:  # pragma: no cover - tested via Fake
    psutil = None  # type: ignore


@dataclass(frozen=True)
class ProcessSnapshot:
    """Immutable snapshot of a process's lightweight metrics."""

    pid: int
    status: str
    cpu_percent: float
    rss_mb: float
    num_threads: int
    num_fds: int
    create_time: float


# Sentinel status names re-exported so trackers do not import ``psutil``.
STATUS_ZOMBIE = "zombie"
STATUS_STOPPED = "stopped"


class ProcessInspector(Protocol):
    """Outbound port: find a process and take a metric snapshot."""

    def available(self) -> bool: ...

    def find(
        self,
        name_pattern: str,
        forced_pid: int = 0,
    ) -> Optional[ProcessSnapshot]: ...

    def refresh(self, pid: int) -> Optional[ProcessSnapshot]: ...


class PsutilProcessInspector:
    """Production adapter backed by ``psutil``."""

    def __init__(self) -> None:
        self._cache: Optional[object] = None
        self._cache_pid: Optional[int] = None

    def available(self) -> bool:
        return psutil is not None

    def _wrap(self, proc) -> Optional[ProcessSnapshot]:  # noqa: ANN001
        if psutil is None:
            return None
        try:
            with proc.oneshot():
                cpu = float(proc.cpu_percent(interval=None))
                mem = proc.memory_info()
                rss_mb = mem.rss / (1024.0 * 1024.0)
                num_threads = int(proc.num_threads())
                try:
                    num_fds = int(proc.num_fds())
                except Exception:
                    num_fds = -1
                status_str = str(proc.status())
                create_time = float(proc.create_time())
                pid = int(proc.pid)
        except Exception:
            self._cache = None
            self._cache_pid = None
            return None
        return ProcessSnapshot(
            pid=pid,
            status=status_str,
            cpu_percent=cpu,
            rss_mb=rss_mb,
            num_threads=num_threads,
            num_fds=num_fds,
            create_time=create_time,
        )

    def find(
        self,
        name_pattern: str,
        forced_pid: int = 0,
    ) -> Optional[ProcessSnapshot]:
        if psutil is None:
            return None
        if forced_pid > 0:
            try:
                proc = psutil.Process(int(forced_pid))
                proc.cpu_percent(interval=None)  # prime
                self._cache = proc
                self._cache_pid = forced_pid
                return self._wrap(proc)
            except Exception:
                return None
        pattern = (name_pattern or "").lower().strip()
        if not pattern:
            return None
        try:
            for proc in psutil.process_iter(attrs=["pid", "name", "cmdline"]):
                info = proc.info
                name = (info.get("name") or "").lower()
                cmd = " ".join(info.get("cmdline") or []).lower()
                if pattern in name or pattern in cmd:
                    proc.cpu_percent(interval=None)
                    self._cache = proc
                    self._cache_pid = int(info["pid"])
                    return self._wrap(proc)
        except Exception:
            return None
        return None

    def refresh(self, pid: int) -> Optional[ProcessSnapshot]:
        if psutil is None or self._cache is None or self._cache_pid != pid:
            return None
        return self._wrap(self._cache)


class FakeProcessInspector:
    """In-memory adapter for tests."""

    def __init__(self, snapshot: Optional[ProcessSnapshot] = None) -> None:
        self._snapshot = snapshot
        self._available = True

    def set_snapshot(self, snapshot: Optional[ProcessSnapshot]) -> None:
        self._snapshot = snapshot

    def set_available(self, available: bool) -> None:
        self._available = available

    def available(self) -> bool:
        return self._available

    def find(
        self,
        name_pattern: str,
        forced_pid: int = 0,
    ) -> Optional[ProcessSnapshot]:
        return self._snapshot

    def refresh(self, pid: int) -> Optional[ProcessSnapshot]:
        if self._snapshot is None or self._snapshot.pid != pid:
            return self._snapshot
        return self._snapshot
