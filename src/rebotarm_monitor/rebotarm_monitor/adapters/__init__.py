"""Adapters: concrete implementations of outbound ports (filesystem, psutil)."""

from .process_info import (
    FakeProcessInspector,
    ProcessInspector,
    ProcessSnapshot,
    PsutilProcessInspector,
)
from .sysfs import FakeSysFsReader, RealSysFsReader, SysFsReader

__all__ = [
    "FakeProcessInspector",
    "FakeSysFsReader",
    "ProcessInspector",
    "ProcessSnapshot",
    "PsutilProcessInspector",
    "RealSysFsReader",
    "SysFsReader",
]
