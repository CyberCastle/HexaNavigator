from __future__ import annotations

from typing import Optional, List
from pydantic import BaseModel


class Waypoint(BaseModel):
    """Mission waypoint definition compatible with QGC mapping."""

    lat: float
    lon: float
    alt: Optional[float] = None
    tolerance_m: float = 1.0
    hold_s: float = 0.0
    speed_mps: Optional[float] = None
    heading_deg: Optional[float] = None


class ImuData(BaseModel):
    stamp: float
    yaw_deg: float
    pitch_deg: float
    roll_deg: float
    gyro_z_dps: float


class GpsData(BaseModel):
    stamp: float
    lat: float
    lon: float
    alt: float
    fix_type: int
    rtk: bool
    speed_mps: float


class LidarScan(BaseModel):
    stamp: float
    angles_rad: List[float]
    ranges_m: List[float]
