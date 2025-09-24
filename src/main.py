from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import time
import logging
from dataclasses import dataclass
from typing import Optional

import yaml

from .utils.logging_setup import setup_logging
from .mavlink import CommandMessage, MavlinkServer, TelemetrySample
from .mission.qgc_plan_loader import load_qgc_plan, load_mavlink_waypoints_txt
from .mission.waypoint_manager import WaypointManager
from .utils.geo import lla_to_enu
from .sensors.imu import Imu, DummyImu
from .sensors.gps import Gps, DummyGps
from .sensors.lidar import Lidar, DummyLidar
from .robot.rpc_client import UartRobotAPI
from .core.state_machine import Navigator


@dataclass
class AppConfig:
    default_cfg_path: str
    sensors_cfg_path: str


def load_yaml(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def build_sensors(sensors_cfg: dict) -> tuple[Imu, Gps, Lidar]:
    imu_type = (sensors_cfg.get("imu") or {}).get("type", "dummy")
    gps_type = (sensors_cfg.get("gps") or {}).get("type", "dummy")
    lidar_type = (sensors_cfg.get("lidar") or {}).get("type", "dummy")

    imu: Imu
    gps: Gps
    lidar: Lidar

    if imu_type == "dummy":
        imu = DummyImu()
    else:
        from .sensors.imu import PiImu  # local import to avoid optional deps

        imu = PiImu()

    if gps_type == "dummy":
        from .sensors.gps import DummyGps

        gps = DummyGps()
    elif gps_type == "serial":
        from .sensors.gps import SerialNmeaGps

        gps = SerialNmeaGps(
            port=sensors_cfg["gps"].get("port", "/dev/ttyUSB0"),
            baudrate=int(sensors_cfg["gps"].get("baudrate", 115200)),
        )
    else:
        from .sensors.gps import DummyGps

        gps = DummyGps()

    if lidar_type == "dummy":
        lidar = DummyLidar()
    elif lidar_type == "rplidar":
        from .sensors.lidar import RplidarA2

        lidar = RplidarA2(
            port=sensors_cfg["lidar"].get("port", "/dev/ttyUSB0"),
            baudrate=int(sensors_cfg["lidar"].get("baudrate", 115200)),
        )
    else:
        lidar = DummyLidar()

    return imu, gps, lidar


def _handle_mavlink_command(cmd: CommandMessage, navigator: Navigator, logger: logging.Logger) -> None:
    try:
        from pymavlink import mavutil  # type: ignore[import]
    except Exception as exc:  # pragma: no cover - optional dependency guard
        logger.debug("pymavlink unavailable to interpret command: %s", exc)
        return

    if cmd.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
        if cmd.params[0] >= 0.5:
            navigator.resume()
            logger.info("Received ARM command via MAVLink")
        else:
            navigator.handle_estop()
            logger.info("Received DISARM command via MAVLink")
    elif cmd.command == mavutil.mavlink.MAV_CMD_MISSION_START:
        navigator.resume()
        logger.info("Received mission start command via MAVLink")
    elif cmd.command == mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH:
        navigator.handle_estop()
        logger.info("Received RTL command -> estop engaged")
    else:
        logger.debug("Unhandled MAVLink command %d", cmd.command)


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(prog="hexapod-autonav")
    sub = parser.add_subparsers(dest="cmd", required=True)
    runp = sub.add_parser("run", help="Run navigation loop")
    runp.add_argument("--plan", required=True, help="Path to QGC .plan or .waypoints file")
    runp.add_argument("--speed", type=float, default=None, help="Target speed m/s")
    runp.add_argument("--gait", type=str, default=None, help="Gait name (e.g., tripod)")
    # UART parameters
    runp.add_argument("--serial-port", type=str, default=os.getenv("SERIAL_PORT", None))
    runp.add_argument("--baudrate", type=int, default=int(os.getenv("BAUDRATE", "115200")))
    runp.add_argument("--log-level", type=str, default=os.getenv("LOG_LEVEL", "INFO"))
    runp.add_argument("--config", type=str, default=None, help="Path to default.yaml override")
    runp.add_argument("--sensors", type=str, default=None, help="Path to sensors.yaml override")

    args = parser.parse_args(argv)

    setup_logging(args.log_level, to_file=True)
    logger = logging.getLogger(__name__)

    # Load configs
    app_cfg = AppConfig(
        default_cfg_path=args.config or os.path.join(os.path.dirname(__file__), "config", "default.yaml"),
        sensors_cfg_path=args.sensors or os.path.join(os.path.dirname(__file__), "config", "sensors.yaml"),
    )

    cfg = load_yaml(app_cfg.default_cfg_path)
    scfg = load_yaml(app_cfg.sensors_cfg_path)

    # Load mission
    if args.plan.endswith(".plan"):
        waypoints = load_qgc_plan(args.plan)
    else:
        waypoints = load_mavlink_waypoints_txt(args.plan)

    wp_mgr = WaypointManager(waypoints)

    # Build sensors
    imu, gps, lidar = build_sensors(scfg)

    # Robot API (UART)
    robot = UartRobotAPI(
        port=args.serial_port,
        baudrate=int(args.baudrate),
        timeout_s=float(cfg.get("uart", {}).get("timeout_s", 2.0)),
    )
    if args.gait:
        try:
            robot.set_gait(args.gait)
        except Exception:
            pass

    # Navigator
    navigator = Navigator(robot=robot, imu=imu, gps=gps, lidar=lidar, wp_mgr=wp_mgr, cfg=cfg)

    # MAVLink server setup (optional)
    mavlink_server: MavlinkServer | None = None
    mav_cfg = cfg.get("mavlink", {}) if isinstance(cfg, dict) else {}
    if bool(mav_cfg.get("enabled", True)):
        try:
            from pymavlink import mavutil  # type: ignore[import]

            mavlink_server = MavlinkServer(
                bind_host=str(mav_cfg.get("bind_host", "0.0.0.0")),
                bind_port=int(mav_cfg.get("bind_port", 14550)),
                system_id=int(mav_cfg.get("system_id", 1)),
                component_id=int(
                    mav_cfg.get(
                        "component_id",
                        mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
                    )
                ),
                heartbeat_rate_hz=float(mav_cfg.get("heartbeat_rate_hz", 1.0)),
                telemetry_rate_hz=float(mav_cfg.get("telemetry_rate_hz", 2.0)),
            )
            mavlink_server.set_mission_waypoints(wp_mgr.waypoints())
            mavlink_server.start()
            if bool(mav_cfg.get("auto_request_mission", True)):
                mavlink_server.request_mission_upload()
            logger.info(
                "MAVLink server listening on %s:%s",
                mavlink_server.bind_host,
                mavlink_server.bind_port,
            )
        except Exception as exc:  # pragma: no cover - defensive log
            logger.warning("Failed to start MAVLink server: %s", exc)
            mavlink_server = None

    running = True

    def handle_sigint(signum, frame):  # noqa: ARG001
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, handle_sigint)
    signal.signal(signal.SIGTERM, handle_sigint)

    rate_hz = float(cfg.get("nav_rate_hz", 10))
    dt = 1.0 / max(rate_hz, 1.0)

    last = time.monotonic()
    while running and not navigator.is_done:
        now = time.monotonic()
        dt = max(1e-3, now - last)
        last = now
        navigator.tick(dt)

        if mavlink_server:
            mission_update = mavlink_server.poll_mission_update()
            while mission_update is not None:
                logger.info("Received MAVLink mission with %d waypoints", len(mission_update))
                navigator.load_mission(mission_update)
                mission_update = mavlink_server.poll_mission_update()

            cmd: CommandMessage | None = mavlink_server.poll_command()
            while cmd is not None:
                _handle_mavlink_command(cmd, navigator, logger)
                cmd = mavlink_server.poll_command()

            gps_data = navigator.last_gps
            imu_data = navigator.last_imu
            if gps_data and imu_data:
                rel_alt = gps_data.alt - (navigator.home_lla[2] if navigator.home_lla else gps_data.alt)
                sample = TelemetrySample(
                    stamp=gps_data.stamp,
                    lat=gps_data.lat,
                    lon=gps_data.lon,
                    alt_amsl_m=gps_data.alt,
                    rel_alt_m=rel_alt,
                    vx_m_s=gps_data.speed_mps,
                    vy_m_s=0.0,
                    vz_m_s=0.0,
                    roll_rad=math.radians(imu_data.roll_deg),
                    pitch_rad=math.radians(imu_data.pitch_deg),
                    yaw_rad=math.radians(imu_data.yaw_deg),
                    fix_type=gps_data.fix_type,
                    satellites_visible=12,
                    hdop=1.0,
                    vdop=1.5,
                )
                mavlink_server.publish_telemetry(sample)

        time.sleep(max(0.0, (1.0 / rate_hz) - (time.monotonic() - now)))

    navigator.handle_estop()  # ensure stop on exit
    if mavlink_server:
        mavlink_server.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
