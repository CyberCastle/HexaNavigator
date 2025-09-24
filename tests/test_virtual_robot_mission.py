from __future__ import annotations

import math
import time
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, List, Tuple

import pytest
from pymavlink import mavutil  # type: ignore[import]
from pytest import CaptureFixture

from src.core.state_machine import Navigator
from src.core.types import GpsData, ImuData, LidarScan, Waypoint
from src.mavlink import MavlinkServer
from src.mission.qgc_plan_loader import load_qgc_plan
from src.mission.waypoint_manager import WaypointManager
from src.robot.api_interface import RobotAPI


class MavlinkTestGcsClient:
    """Minimal MAVLink client to upload missions to the server."""

    def __init__(self, *, port: int, system_id: int, component_id: int) -> None:
        self._conn = mavutil.mavlink_connection(
            f"udpout:127.0.0.1:{port}",
            source_system=system_id,
            source_component=component_id,
        )

    def close(self) -> None:
        try:
            self._conn.close()  # type: ignore[attr-defined]
        except Exception:  # pragma: no cover - defensive
            pass

    def send_heartbeat(self) -> None:
        self._conn.mav.heartbeat_send(  # type: ignore[attr-defined]
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )

    def wait_for(self, message_type: str, timeout: float = 5.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self._conn.recv_match(type=message_type, blocking=False)
            if msg is not None:
                return msg
            time.sleep(0.01)
        return None

    def send_mission_count(self, *, count: int) -> None:
        self._conn.mav.mission_count_send(  # type: ignore[attr-defined]
            target_system=1,
            target_component=1,
            count=count,
        )

    def send_mission_item(self, *, seq: int, waypoint: Waypoint) -> None:
        alt = 0.0 if waypoint.alt is None else float(waypoint.alt)
        self._conn.mav.mission_item_int_send(  # type: ignore[attr-defined]
            target_system=1,
            target_component=1,
            seq=seq,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            current=1 if seq == 0 else 0,
            autocontinue=1,
            param1=waypoint.hold_s,
            param2=waypoint.tolerance_m,
            param3=0.0,
            param4=waypoint.heading_deg or 0.0,
            x=int(waypoint.lat * 1e7),
            y=int(waypoint.lon * 1e7),
            z=alt,
        )


class VirtualRobot(RobotAPI):
    """RobotAPI implementation that records the issued commands."""

    def __init__(self) -> None:
        self.history: List[Tuple[str, Tuple[Any, ...], dict[str, Any]]] = []

    def _record(self, name: str, *args: Any, **kwargs: Any) -> None:
        self.history.append((name, args, kwargs))

    def forward(self, speed: float) -> None:
        self._record("forward", speed)

    def backward(self, speed: float) -> None:
        self._record("backward", speed)

    def turn_left(self, rate: float) -> None:
        self._record("turn_left", rate)

    def turn_right(self, rate: float) -> None:
        self._record("turn_right", rate)

    def strafe_left(self, speed: float) -> None:
        self._record("strafe_left", speed)

    def strafe_right(self, speed: float) -> None:
        self._record("strafe_right", speed)

    def stop(self) -> None:
        self._record("stop")

    def set_velocity(self, v: float) -> None:
        self._record("set_velocity", v)

    def set_gait(self, name: str) -> None:
        self._record("set_gait", name)

    def set_pose(self, **kwargs: Any) -> None:
        self._record("set_pose", **kwargs)


@dataclass
class ScriptedImu:
    yaw_deg: float = 0.0

    def start(self) -> None:
        """No-op start to satisfy Imu protocol."""

    def stop(self) -> None:
        """No-op stop to satisfy Imu protocol."""

    def set_yaw(self, yaw: float) -> None:
        self.yaw_deg = yaw

    def read(self) -> ImuData:
        return ImuData(
            stamp=time.time(),
            yaw_deg=self.yaw_deg,
            pitch_deg=0.0,
            roll_deg=0.0,
            gyro_z_dps=0.0,
        )


@dataclass
class ScriptedGps:
    lat: float
    lon: float
    alt: float
    speed_mps: float = 0.0

    def set_position(self, lat: float, lon: float, alt: float | None = None, speed_mps: float | None = None) -> None:
        self.lat = lat
        self.lon = lon
        if alt is not None:
            self.alt = alt
        if speed_mps is not None:
            self.speed_mps = speed_mps

    def read(self) -> GpsData:
        return GpsData(
            stamp=time.time(),
            lat=self.lat,
            lon=self.lon,
            alt=self.alt,
            fix_type=3,
            rtk=False,
            speed_mps=self.speed_mps,
        )

    def has_fix(self) -> bool:
        return True


class ScriptedLidar:
    def read(self) -> LidarScan:
        return LidarScan(
            stamp=time.time(),
            angles_rad=[0.0, math.pi / 2, math.pi, 3 * math.pi / 2],
            ranges_m=[5.0, 5.0, 5.0, 5.0],
        )


@pytest.mark.integration
def test_virtual_robot_executes_mavlink_plan(capsys: CaptureFixture[str]) -> None:
    port = 14670
    server = MavlinkServer(bind_host="127.0.0.1", bind_port=port, telemetry_rate_hz=5.0)
    server.start()
    try:
        gcs = MavlinkTestGcsClient(
            port=port,
            system_id=220,
            component_id=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
        )
        try:
            gcs.send_heartbeat()
            time.sleep(0.2)

            server.request_mission_upload()
            request = gcs.wait_for("MISSION_REQUEST_LIST", timeout=5.0)
            assert request is not None

            plan_path = Path(__file__).with_name("test.plan")
            plan = load_qgc_plan(str(plan_path))
            assert plan, "Plan file must contain at least one waypoint"
            gcs.send_mission_count(count=len(plan))

            for seq, wp in enumerate(plan):
                gcs.send_mission_item(seq=seq, waypoint=wp)
                if seq < len(plan) - 1:
                    follow_up = gcs.wait_for("MISSION_REQUEST_INT", timeout=5.0)
                    assert follow_up is not None

            ack = gcs.wait_for("MISSION_ACK", timeout=5.0)
            assert ack is not None

            mission = server.poll_mission_update(timeout=2.0)
            assert mission is not None
            assert len(mission) == len(plan)
            for expected, actual in zip(plan, mission):
                assert math.isclose(actual.lat, expected.lat, abs_tol=1e-7)
                assert math.isclose(actual.lon, expected.lon, abs_tol=1e-7)
                if expected.alt is not None:
                    assert actual.alt is not None
                    assert math.isclose(actual.alt, expected.alt, abs_tol=1e-3)
                assert math.isclose(actual.tolerance_m, expected.tolerance_m, rel_tol=0, abs_tol=1e-3)
                assert math.isclose(actual.hold_s, expected.hold_s, rel_tol=0, abs_tol=1e-3)
                if expected.heading_deg is None:
                    assert actual.heading_deg in (None, 0.0)
                else:
                    assert actual.heading_deg is not None
                    assert math.isclose(actual.heading_deg, expected.heading_deg, abs_tol=1e-3)

            with open(plan_path, "r", encoding="utf-8") as fp:
                plan_data = json.load(fp)
            home_raw = plan_data.get("mission", {}).get(
                "plannedHomePosition",
                [mission[0].lat, mission[0].lon, mission[0].alt or 0.0],
            )
            home_lat = float(home_raw[0]) if len(home_raw) > 0 else float(mission[0].lat)
            home_lon = float(home_raw[1]) if len(home_raw) > 1 else float(mission[0].lon)
            home_alt = float(home_raw[2]) if len(home_raw) > 2 and home_raw[2] is not None else float(mission[0].alt or 0.0)

            robot = VirtualRobot()
            imu = ScriptedImu()
            first_wp = mission[0]
            gps = ScriptedGps(lat=home_lat, lon=home_lon, alt=home_alt)
            lidar = ScriptedLidar()
            wp_mgr = WaypointManager([])
            cfg = {
                "lookahead_m": 0.5,
                "safety_radius_m": 0.4,
                "vel_min": 0.05,
                "vel_max": 0.2,
                "heading_deadband_deg": 10,
                "turn_rate_max_dps": 45,
            }
            navigator = Navigator(robot, imu, gps, lidar, wp_mgr, cfg)
            navigator.load_mission(mission)

            current_lat = gps.lat
            current_lon = gps.lon
            current_alt = gps.alt

            def announce(message: str) -> None:
                with capsys.disabled():
                    print(f"[VirtualRobot] {message}")

            announce(
                "Departing launch at lat={:.6f}, lon={:.6f} toward waypoint 1 at lat={:.6f}, lon={:.6f}".format(
                    home_lat,
                    home_lon,
                    first_wp.lat,
                    first_wp.lon,
                )
            )

            for idx, wp in enumerate(mission):
                # introduce a heading error first to exercise turning, then align and reach the waypoint
                imu.set_yaw(90.0)
                gps.set_position(current_lat, current_lon, current_alt, speed_mps=0.0)
                navigator.tick(0.1)

                imu.set_yaw(0.0)
                announce(f"Approaching waypoint {idx + 1}/{len(mission)} " f"at lat={wp.lat:.6f}, lon={wp.lon:.6f}")
                gps.set_position(
                    wp.lat,
                    wp.lon,
                    wp.alt if wp.alt is not None else current_alt,
                    speed_mps=0.1,
                )
                navigator.tick(0.1)

                current_lat = wp.lat
                current_lon = wp.lon
                current_alt = wp.alt if wp.alt is not None else current_alt

            # Simulate the operator commanding a return-to-launch after final waypoint
            announce(
                "Returning from waypoint {} at lat={:.6f}, lon={:.6f} to launch at lat={:.6f}, lon={:.6f}".format(
                    len(mission),
                    current_lat,
                    current_lon,
                    home_lat,
                    home_lon,
                )
            )

            commands = [name for name, *_ in robot.history]
            # First command comes from load_mission() stop; inspect the rest for navigation activity
            navigation_commands = commands[1:]
            assert any(cmd == "forward" for cmd in navigation_commands)
            assert navigation_commands[-1] == "stop"
            assert navigator.is_done
            assert wp_mgr.remaining() == 0

            summary_lines = [
                f"Virtual robot executed {len(navigation_commands)} navigation commands",
                "Command stream: " + ", ".join(navigation_commands[:20]) + ("..." if len(navigation_commands) > 20 else ""),
            ]
            for line in summary_lines:
                announce(line)
        finally:
            gcs.close()
    finally:
        server.stop()
