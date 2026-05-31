"""Bridge so ``colcon test`` (which runs ``python -m unittest``) executes the
pytest suite under ``test/``.

ament_python packages without the ``ament_pytest`` plugin fall back to a
plain ``python -m unittest -v`` invocation that only discovers
``unittest.TestCase`` classes. Our tests are pytest-style (plain functions,
fixtures), so unittest finds zero tests. This single TestCase delegates to
pytest, keeping ``colcon test`` integration without rewriting the suite.
"""

from __future__ import annotations

import unittest
from pathlib import Path


class PytestSuite(unittest.TestCase):
    """Run the pytest suite as a single unittest test."""

    def test_pytest_suite_passes(self) -> None:
        import pytest

        test_dir = Path(__file__).resolve().parent / "test"
        exit_code = pytest.main(["-q", str(test_dir)])
        self.assertEqual(
            int(exit_code),
            0,
            f"pytest reported exit code {int(exit_code)}",
        )
