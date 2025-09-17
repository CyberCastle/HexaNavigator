from __future__ import annotations

import json
from typing import List

from ..core.types import Waypoint


def load_qgc_plan(path: str) -> List[Waypoint]:
    """Parse QGroundControl .plan (v2/v3 JSON) into a list of Waypoint.

    Only MAV_CMD_NAV_WAYPOINT items are converted. Unsupported items are ignored.
    """
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    items = data.get("mission", {}).get("items", [])
    wps: List[Waypoint] = []
    for it in items:
        cmd = it.get("command") or it.get("Command")
        if cmd not in (16, "MAV_CMD_NAV_WAYPOINT"):
            continue
        # Extract lat/lon/alt
        lat = it.get("coordinate", [None, None, None])[0]
        lon = it.get("coordinate", [None, None, None])[1]
        alt = it.get("coordinate", [None, None, None])[2]
        if lat is None:
            lat = it.get("params", [None, None, None, None, None, None, None])[4]
        if lon is None:
            lon = it.get("params", [None, None, None, None, None, None, None])[5]
        if alt is None:
            alt = it.get("params", [None, None, None, None, None, None, None])[6]

        tol = it.get("AcceptanceRadius", it.get("param2", 1.0))
        hold = it.get("Hold", it.get("param1", 0.0))
        speed = it.get("Speed", None)
        heading = it.get("Yaw", None)

        if lat is None or lon is None:
            continue
        wps.append(
            Waypoint(
                lat=float(lat),
                lon=float(lon),
                alt=float(alt) if alt is not None else None,
                tolerance_m=float(tol) if tol is not None else 1.0,
                hold_s=float(hold) if hold is not None else 0.0,
                speed_mps=float(speed) if speed is not None else None,
                heading_deg=float(heading) if heading is not None else None,
            )
        )
    return wps


def load_mavlink_waypoints_txt(path: str) -> List[Waypoint]:
    """Parse MAVLink plain text waypoints format (*.waypoints). Minimal support.

    Ignores non-NAV_WAYPOINT lines. Columns are as in QGC WPL: index, current, frame,
    command, param1..4, x(lat), y(lon), z(alt), autocontinue
    """
    wps: List[Waypoint] = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split("\t")
            if len(parts) < 12:
                parts = line.split(",")  # tolerate CSV-like
            if len(parts) < 12:
                continue
            try:
                cmd = int(parts[3])
            except Exception:
                continue
            if cmd != 16:
                continue
            lat = float(parts[8])
            lon = float(parts[9])
            alt = float(parts[10])
            wps.append(Waypoint(lat=lat, lon=lon, alt=alt))
    return wps
