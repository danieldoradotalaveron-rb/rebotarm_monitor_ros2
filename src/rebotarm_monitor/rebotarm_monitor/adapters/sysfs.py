"""Sysfs port + adapters.

The port is the minimal API a tracker needs to read kernel-exposed counters.
A real adapter reads ``/sys`` directly; a fake adapter lets unit tests inject
controlled values without touching disk.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional, Protocol


class SysFsReader(Protocol):
    """Outbound port for reading kernel-exposed text/int files."""

    def is_dir(self, path: str) -> bool: ...

    def read_int(self, path: str) -> Optional[int]: ...

    def read_text(self, path: str) -> Optional[str]: ...


class RealSysFsReader:
    """Production adapter that reads directly from the host filesystem."""

    def is_dir(self, path: str) -> bool:
        return Path(path).is_dir()

    def read_int(self, path: str) -> Optional[int]:
        try:
            return int(Path(path).read_text().strip())
        except (OSError, ValueError):
            return None

    def read_text(self, path: str) -> Optional[str]:
        try:
            return Path(path).read_text().strip()
        except OSError:
            return None


class FakeSysFsReader:
    """In-memory adapter for unit tests.

    Construct with a mapping of paths to string contents. Directories are
    derived from the prefixes of registered files.
    """

    def __init__(self, files: Optional[dict[str, str]] = None) -> None:
        self._files: dict[str, str] = dict(files or {})

    def set(self, path: str, value: str) -> None:
        self._files[path] = value

    def remove(self, path: str) -> None:
        self._files.pop(path, None)

    def is_dir(self, path: str) -> bool:
        prefix = path.rstrip("/") + "/"
        return any(key.startswith(prefix) for key in self._files)

    def read_int(self, path: str) -> Optional[int]:
        raw = self._files.get(path)
        if raw is None:
            return None
        try:
            return int(raw.strip())
        except ValueError:
            return None

    def read_text(self, path: str) -> Optional[str]:
        raw = self._files.get(path)
        return None if raw is None else raw.strip()
