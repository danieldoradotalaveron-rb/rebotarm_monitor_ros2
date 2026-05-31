"""Sliding-window message-rate accumulator."""

from __future__ import annotations

import time
from dataclasses import dataclass, field


@dataclass
class RateWindow:
    """Counts messages since ``window_start`` and yields a measured Hz value."""

    msg_count: int = 0
    window_start: float = field(default_factory=time.monotonic)

    def measured_hz(self) -> float:
        elapsed = time.monotonic() - self.window_start
        if elapsed <= 0.0:
            return 0.0
        return self.msg_count / elapsed

    def reset(self) -> None:
        self.msg_count = 0
        self.window_start = time.monotonic()
