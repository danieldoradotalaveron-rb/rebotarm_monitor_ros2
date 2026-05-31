"""Device-path port + adapters for serial/TTY link checks."""

from __future__ import annotations

import os
import stat
from pathlib import Path
from typing import Optional, Protocol, Set, Union


class DevicePathInspector(Protocol):
    """Outbound port: inspect a character device path on the host."""

    def exists(self, path: str) -> bool: ...

    def resolve(self, path: str) -> Optional[str]: ...

    def is_char_device(self, path: str) -> bool: ...

    def is_readable_writable(self, path: str) -> bool: ...


class RealDevicePathInspector:
    """Production adapter using ``pathlib`` and ``os.access``."""

    def exists(self, path: str) -> bool:
        return Path(path).exists()

    def resolve(self, path: str) -> Optional[str]:
        try:
            return str(Path(path).resolve())
        except OSError:
            return None

    def is_char_device(self, path: str) -> bool:
        try:
            return stat.S_ISCHR(Path(path).stat().st_mode)
        except OSError:
            return False

    def is_readable_writable(self, path: str) -> bool:
        return os.access(path, os.R_OK | os.W_OK)


class FakeDevicePathInspector:
    """In-memory adapter for unit tests."""

    def __init__(
        self,
        *,
        exists: Union[bool, Set[str], None] = True,
        resolved: Optional[str] = None,
        char_device: bool = True,
        readable_writable: bool = True,
    ) -> None:
        if exists is None:
            self._exists_paths: Set[str] = set()
        elif isinstance(exists, bool):
            self._exists_all = exists
            self._exists_paths = None
        else:
            self._exists_all = None
            self._exists_paths = set(exists)
        self._resolved = resolved
        self._char_device = char_device
        self._readable_writable = readable_writable

    def exists(self, path: str) -> bool:
        if self._exists_paths is not None:
            return path in self._exists_paths
        return bool(self._exists_all)

    def resolve(self, path: str) -> Optional[str]:
        if not self.exists(path):
            return None
        return self._resolved or path

    def is_char_device(self, path: str) -> bool:
        return self.exists(path) and self._char_device

    def is_readable_writable(self, path: str) -> bool:
        return self.exists(path) and self._readable_writable
