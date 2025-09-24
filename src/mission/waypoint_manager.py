from __future__ import annotations

from typing import List, Optional, Tuple

from ..core.types import Waypoint


class WaypointManager:
    """Simple waypoint manager that advances when within tolerance."""

    def __init__(self, waypoints: List[Waypoint]):
        self._wps = list(waypoints)
        self._idx = 0

    def current(self) -> Optional[Waypoint]:
        return self._wps[self._idx] if self._idx < len(self._wps) else None

    def remaining(self) -> int:
        return max(0, len(self._wps) - self._idx)

    def waypoints(self) -> List[Waypoint]:
        return list(self._wps)

    def replace_waypoints(self, waypoints: List[Waypoint]) -> None:
        self._wps = list(waypoints)
        self._idx = 0

    def reset(self) -> None:
        self._idx = 0

    def advance_if_reached(self, pos_enu: Tuple[float, float], tolerance: float | None = None) -> bool:
        wp = self.current()
        if not wp:
            return False
        tol = float(tolerance if tolerance is not None else wp.tolerance_m)
        dx = pos_enu[0]
        dy = pos_enu[1]
        dist = (dx * dx + dy * dy) ** 0.5
        if dist <= tol:
            self._idx += 1
            return True
        return False
