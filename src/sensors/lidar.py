from __future__ import annotations

import math
import time
from typing import Protocol, List, Tuple

import numpy as np

from ..core.types import LidarScan


class Lidar(Protocol):
    def read(self) -> LidarScan: ...


class DummyLidar:
    def read(self) -> LidarScan:
        # Produce a clear scan (no obstacles) 360 degrees
        angles = [math.radians(a) for a in range(0, 360, 2)]
        ranges = [5.0 for _ in angles]
        return LidarScan(stamp=time.time(), angles_rad=angles, ranges_m=ranges)


class RplidarA2(DummyLidar):
    """Stub for RPLIDAR A2 driver using rplidar package."""

    def __init__(self, port: str, baudrate: int = 115200) -> None:
        self._port = port
        self._baud = baudrate
        # TODO: connect to RPLIDAR


def scan_to_occupancy(scan: LidarScan, grid_cfg: dict) -> np.ndarray:
    """Project scan to a simple occupancy grid.

    Args:
        scan: LidarScan
        grid_cfg: {"size": (rows, cols), "resolution": m_per_cell, "origin": (row0, col0)}
    Returns:
        np.ndarray[rows, cols] with 0=free, 1=occupied
    """
    rows, cols = grid_cfg.get("size", (100, 100))
    res = float(grid_cfg.get("resolution", 0.1))
    grid = np.zeros((rows, cols), dtype=np.uint8)
    orow, ocol = grid_cfg.get("origin", (rows // 2, cols // 4))

    for ang, rng in zip(scan.angles_rad, scan.ranges_m):
        if not (0.05 <= rng <= 10.0):
            continue
        ex = math.cos(ang) * rng
        ey = math.sin(ang) * rng
        r = int(orow - ey / res)
        c = int(ocol + ex / res)
        if 0 <= r < rows and 0 <= c < cols:
            grid[r, c] = 1
    return grid


def polar_sectorization(scan: LidarScan, safety_radius_m: float, sector_deg: int = 10) -> dict:
    """Compute minimal obstacle distance per angular sector centered forward.

    Returns a dict: {sector_center_deg: min_range_m}
    """
    sectors = {}
    for ang, rng in zip(scan.angles_rad, scan.ranges_m):
        deg = (math.degrees(ang) + 360.0) % 360.0
        # We define 0 deg as forward (x+). Adjust if needed for sensor mounting.
        sector = int((deg + sector_deg / 2) // sector_deg) * sector_deg
        sectors.setdefault(sector, [])
        sectors[sector].append(rng)
    result = {k: min(v) if v else float("inf") for k, v in sectors.items()}
    return result
