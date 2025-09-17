from __future__ import annotations

from math import atan2, degrees, radians, sin, cos
from typing import Tuple
from geographiclib.geodesic import Geodesic


def wrap_angle(angle_deg: float) -> float:
    """Wrap an angle in degrees to [-180, 180)."""
    a = (angle_deg + 180.0) % 360.0 - 180.0
    return -180.0 if a == 180.0 else a


def bearing_deg(p0: Tuple[float, float], p1: Tuple[float, float]) -> float:
    """Initial bearing from p0(lat,lon) to p1(lat,lon) in degrees [0,360)."""
    lat1, lon1 = map(radians, p0)
    lat2, lon2 = map(radians, p1)
    dlon = lon2 - lon1
    x = sin(dlon) * cos(lat2)
    y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)
    brng = degrees(atan2(x, y))
    return (brng + 360.0) % 360.0


def lla_to_enu(
    home_lla: Tuple[float, float, float], target_lla: Tuple[float, float, float]
) -> Tuple[float, float, float]:
    """Convert LLA to local ENU with origin at home using GeographicLib.

    Args:
        home_lla: (lat, lon, alt) in degrees/meters.
        target_lla: (lat, lon, alt) in degrees/meters.

    Returns:
        (e, n, u) in meters.
    """
    geod = Geodesic.WGS84
    g = geod.Inverse(home_lla[0], home_lla[1], target_lla[0], target_lla[1])
    s12 = g["s12"]
    azi1 = radians(g["azi1"])  # forward azimuth from home to target
    # Decompose along azimuth into EN
    north = s12 * cos(azi1)
    east = s12 * sin(azi1)
    up = (target_lla[2] if target_lla[2] is not None else 0.0) - (
        home_lla[2] if home_lla[2] is not None else 0.0
    )
    return float(east), float(north), float(up)
