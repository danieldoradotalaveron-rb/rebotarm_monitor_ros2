"""Format observed values against configured limits for diagnostic messages."""

from __future__ import annotations


def format_abs_vs_limit(
    value: float,
    limit: float,
    unit: str,
    *,
    decimals: int = 1,
) -> str:
    """``|value|/limit unit`` for compact rqt ``DiagnosticStatus.message`` lines."""
    return f"{abs(value):.{decimals}f}/{limit:.{decimals}f} {unit}"
