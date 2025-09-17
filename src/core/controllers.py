from __future__ import annotations

import math
from enum import Enum
from typing import Iterable, Tuple, List

from ..utils.geo import wrap_angle


class Avoidance(Enum):
    CLEAR = 0
    TURN_LEFT = 1
    TURN_RIGHT = 2
    STOP = 3


def pure_pursuit(
    current_xy: Tuple[float, float],
    heading_deg: float,
    path_points: List[Tuple[float, float]],
    lookahead_m: float,
) -> tuple[float, float]:
    """Minimal pure pursuit: aim at the first path point beyond lookahead distance.

    Returns target_heading_deg and target_speed (normalized 0..1).
    """
    if not path_points:
        return heading_deg, 0.0
    cx, cy = current_xy
    # find first point beyond lookahead
    target = path_points[-1]
    for px, py in path_points:
        d = math.hypot(px - cx, py - cy)
        if d >= lookahead_m:
            target = (px, py)
            break
    # desired heading
    desired = math.degrees(math.atan2(target[1] - cy, target[0] - cx))
    err = wrap_angle(desired - heading_deg)
    # speed scale decreases with heading error
    speed_scale = max(0.0, 1.0 - abs(err) / 180.0)
    return desired, speed_scale


def avoidance_decision(
    sectors: dict, safety_radius_m: float, forward_sector_width_deg: int = 60
) -> Avoidance:
    """Simple sector-based avoidance.

    Looks at +/- forward_sector_width/2 around 0 deg. If any sector < safety_radius, decide turns.
    """
    # consider forward sectors centered near 0 and near 360 wrap
    forward_keys = [
        k
        for k in sectors.keys()
        if k <= forward_sector_width_deg // 2 or k >= 360 - forward_sector_width_deg // 2
    ]
    if not forward_keys:
        return Avoidance.CLEAR
    min_front = min(sectors[k] for k in forward_keys if k in sectors)
    if min_front >= safety_radius_m:
        return Avoidance.CLEAR
    # decide left vs right bias based on side distances
    left_keys = [k for k in sectors.keys() if 0 < k < 180]
    right_keys = [k for k in sectors.keys() if 180 < k < 360]
    left_min = min([sectors[k] for k in left_keys], default=float("inf"))
    right_min = min([sectors[k] for k in right_keys], default=float("inf"))
    if left_min > right_min:
        return Avoidance.TURN_LEFT
    if right_min > left_min:
        return Avoidance.TURN_RIGHT
    return Avoidance.STOP


def fuse_and_act(
    robot,
    target_heading_deg: float,
    current_heading_deg: float,
    target_speed: float,
    avoidance: Avoidance,
    cfg: dict,
) -> None:
    """Map heading/speed targets and avoidance decision to RobotAPI commands."""
    heading_err = wrap_angle(target_heading_deg - current_heading_deg)
    turn_rate_max = float(cfg.get("turn_rate_max_dps", 45.0))
    speed_max = float(cfg.get("vel_max", 0.2))

    if avoidance == Avoidance.STOP:
        robot.stop()
        return
    if avoidance == Avoidance.TURN_LEFT:
        robot.turn_left(turn_rate_max * 0.5)
        return
    if avoidance == Avoidance.TURN_RIGHT:
        robot.turn_right(turn_rate_max * 0.5)
        return

    # CLEAR
    if abs(heading_err) > float(cfg.get("heading_deadband_deg", 10.0)):
        if heading_err > 0:
            robot.turn_left(min(turn_rate_max, abs(heading_err)))
        else:
            robot.turn_right(min(turn_rate_max, abs(heading_err)))
    else:
        speed = min(speed_max, max(float(cfg.get("vel_min", 0.05)), target_speed * speed_max))
        robot.forward(speed)
