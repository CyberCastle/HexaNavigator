from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import time
from dataclasses import dataclass
from typing import Optional

import yaml

from .utils.logging_setup import setup_logging
from .mission.qgc_plan_loader import load_qgc_plan, load_mavlink_waypoints_txt
from .mission.waypoint_manager import WaypointManager
from .utils.geo import lla_to_enu
from .sensors.imu import Imu, DummyImu
from .sensors.gps import Gps, DummyGps
from .sensors.lidar import Lidar, DummyLidar
from .robot.rpc_client import HttpRpcRobotAPI
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


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(prog="hexapod-autonav")
    sub = parser.add_subparsers(dest="cmd", required=True)
    runp = sub.add_parser("run", help="Run navigation loop")
    runp.add_argument("--plan", required=True, help="Path to QGC .plan or .waypoints file")
    runp.add_argument("--speed", type=float, default=None, help="Target speed m/s")
    runp.add_argument("--gait", type=str, default=None, help="Gait name (e.g., tripod)")
    runp.add_argument(
        "--rpc-url", type=str, default=os.getenv("RPC_BASE_URL", "http://localhost:8000")
    )
    runp.add_argument("--log-level", type=str, default=os.getenv("LOG_LEVEL", "INFO"))
    runp.add_argument("--config", type=str, default=None, help="Path to default.yaml override")
    runp.add_argument("--sensors", type=str, default=None, help="Path to sensors.yaml override")

    args = parser.parse_args(argv)

    setup_logging(args.log_level, to_file=True)

    # Load configs
    app_cfg = AppConfig(
        default_cfg_path=args.config
        or os.path.join(os.path.dirname(__file__), "config", "default.yaml"),
        sensors_cfg_path=args.sensors
        or os.path.join(os.path.dirname(__file__), "config", "sensors.yaml"),
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

    # Robot API
    robot = HttpRpcRobotAPI(
        base_url=args.rpc_url, timeout_s=float(cfg.get("rpc", {}).get("timeout_s", 2.0))
    )
    if args.gait:
        try:
            robot.set_gait(args.gait)
        except Exception:
            pass

    # Navigator
    navigator = Navigator(robot=robot, imu=imu, gps=gps, lidar=lidar, wp_mgr=wp_mgr, cfg=cfg)

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
        time.sleep(max(0.0, (1.0 / rate_hz) - (time.monotonic() - now)))

    navigator.handle_estop()  # ensure stop on exit
    return 0


if __name__ == "__main__":
    sys.exit(main())
