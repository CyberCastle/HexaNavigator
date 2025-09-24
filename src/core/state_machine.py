from __future__ import annotations

import logging
import time
from enum import Enum
from typing import Optional, Tuple

from ..core.types import GpsData, ImuData, Waypoint
from ..utils.geo import lla_to_enu
from ..sensors.imu import Imu
from ..sensors.gps import Gps
from ..sensors.lidar import Lidar, polar_sectorization
from ..mission.waypoint_manager import WaypointManager
from .controllers import pure_pursuit, avoidance_decision, fuse_and_act, Avoidance

log = logging.getLogger(__name__)


class State(Enum):
    IDLE = 0
    NAVIGATING = 1
    AVOIDING = 2
    PAUSED = 3
    COMPLETED = 4
    ABORTED = 5


class Navigator:
    def __init__(self, robot, imu: Imu, gps: Gps, lidar: Lidar, wp_mgr: WaypointManager, cfg: dict):
        self.robot = robot
        self.imu = imu
        self.gps = gps
        self.lidar = lidar
        self.wp_mgr = wp_mgr
        self.cfg = cfg
        self.state = State.IDLE
        self.home_lla: Optional[tuple[float, float, float]] = None
        self.is_done = False
        self.last_imu: Optional[ImuData] = None
        self.last_gps: Optional[GpsData] = None
        self.last_lidar = None

    def tick(self, dt: float) -> None:
        if self.is_done:
            return
        # Read sensors
        imu = self.imu.read()
        gps = self.gps.read()
        scan = self.lidar.read()
        self.last_imu = imu
        self.last_gps = gps
        self.last_lidar = scan

        # Initialize home
        if self.home_lla is None and self.gps.has_fix():
            self.home_lla = (gps.lat, gps.lon, gps.alt)
            self.state = State.NAVIGATING if self.wp_mgr.current() else State.COMPLETED
            log.info("Home fix set at lat=%.6f lon=%.6f alt=%.1f", *self.home_lla)

        if self.state in (State.COMPLETED, State.ABORTED, State.PAUSED):
            return

        if self.home_lla is None:
            log.debug("Waiting for home fix...")
            return

        # Compute current ENU relative to current waypoint for advance check
        wp = self.wp_mgr.current()
        if wp is None:
            self.state = State.COMPLETED
            self.is_done = True
            self.robot.stop()
            log.info("Mission complete")
            return

        e, n, u = lla_to_enu(self.home_lla, (wp.lat, wp.lon, wp.alt or self.home_lla[2]))
        # Note: using target ENU relative to home as the desired path point; current pos assumed near home for dummy
        # In real use, compute current ENU from current GPS: lla_to_enu(self.home_lla, (gps.lat, gps.lon, gps.alt))
        curr_e, curr_n, _ = lla_to_enu(self.home_lla, (gps.lat, gps.lon, gps.alt))

        path_points = [(e, n)]
        target_heading, target_speed = pure_pursuit((curr_e, curr_n), imu.yaw_deg, path_points, float(self.cfg.get("lookahead_m", 0.5)))

        sectors = polar_sectorization(scan, safety_radius_m=float(self.cfg.get("safety_radius_m", 0.4)))
        decision = avoidance_decision(sectors, float(self.cfg.get("safety_radius_m", 0.4)))

        if decision in (Avoidance.TURN_LEFT, Avoidance.TURN_RIGHT, Avoidance.STOP):
            self.state = State.AVOIDING
        else:
            if self.state == State.AVOIDING:
                self.state = State.NAVIGATING

        fuse_and_act(self.robot, target_heading, imu.yaw_deg, target_speed, decision, self.cfg)

        # Advance waypoint if reached (distance in ENU)
        vec_e = e - curr_e
        vec_n = n - curr_n
        reached = self.wp_mgr.advance_if_reached((vec_e, vec_n), tolerance=None)
        if reached:
            log.info("Reached waypoint, advancing. Remaining: %d", self.wp_mgr.remaining())
            if self.wp_mgr.current() is None:
                self.state = State.COMPLETED
                self.is_done = True
                self.robot.stop()

    def handle_estop(self) -> None:
        self.robot.stop()
        if not self.is_done:
            self.state = State.PAUSED

    def load_mission(self, waypoints: list[Waypoint]) -> None:
        self.wp_mgr.replace_waypoints(waypoints)
        self.home_lla = None
        self.is_done = False
        self.state = State.IDLE
        self.robot.stop()

    def resume(self) -> None:
        if self.wp_mgr.current():
            self.state = State.NAVIGATING
            self.is_done = False
