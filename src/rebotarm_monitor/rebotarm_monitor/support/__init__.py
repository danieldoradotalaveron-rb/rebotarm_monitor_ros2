"""Cross-cutting helpers (diagnostics formatting, rate windows)."""

from .diagnostics import is_finite, kv, max_level
from .rate_window import RateWindow

__all__ = ["RateWindow", "is_finite", "kv", "max_level"]
