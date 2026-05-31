"""Domain layer: pure logic, framework-free types and contracts."""

from .tracker import HealthTracker, TrackerContext

__all__ = ["HealthTracker", "TrackerContext"]
