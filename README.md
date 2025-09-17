# Hexapod AutoNav

Autonomous waypoint navigation for a Raspberry Pi–mounted hexapod robot using QGroundControl missions, IMU, GPS/RTK, and LiDAR. No move_to(x,y) is assumed; the controller translates plans into directional RPC commands (forward/backward/turn/stop).

-   Mission input: QGC .plan (JSON) and MAVLink plain text .waypoints (optional)
-   Sensors: IMU, GPS/RTK, LiDAR (camera optional)
-   Planner: Pure Pursuit + reactive obstacle avoidance (sectorization/VFH-lite)
-   Control: HTTP/JSON-RPC client for hexapod directional API
-   State machine: IDLE → NAVIGATING ↔ AVOIDING → COMPLETED/ABORTED/PAUSED

## Hardware target

-   Raspberry Pi (Bookworm/Raspberry Pi OS recommended)
-   IMU (I2C/SPI), GPS/RTK (UART/USB), 2D LiDAR (USB/UART)
-   Optional camera (Picamera2/OpenCV)

## Install (Python 3.12)

1. System packages (recommended)

```bash
# On Raspberry Pi
sudo apt update
sudo apt install -y python3-pip python3-venv
```

2. Create and activate venv

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
```

3. Install package (editable)

```bash
pip install -e .
```

Dependencies you'll likely need to add in your pyproject if missing:

-   geographiclib
-   numpy
-   pydantic
-   pyyaml
-   requests
-   Optional: pynmea2, pyserial, rplidar, opencv-python or picamera2

## Run

Default (dummy sensors), using the root CLI shim:

```bash
python -m main run --plan mission.plan --speed 0.15 --gait tripod
```

Or using the package module directly:

```bash
python -m hexapod_autonav.main run --plan mission.plan --speed 0.15 --gait tripod
```

Configuration:

-   Edit `src/hexapod_autonav/config/default.yaml` for navigation parameters.
-   Edit `src/hexapod_autonav/config/sensors.yaml` to select adapters (dummy/real) and ports.

## RPC integration

Set the base URL via `--rpc-url` or environment variable `RPC_BASE_URL`. The default `HttpRpcRobotAPI` expects endpoints like:

-   POST /cmd/forward {"speed": 0.15}
-   POST /cmd/turn_left {"rate": 20}
-   POST /cmd/stop {}
-   POST /config/set_velocity {"v": 0.2}
-   POST /config/set_gait {"name": "tripod"}

Adapt in `src/hexapod_autonav/robot/rpc_client.py` as needed.

## Extend drivers

Implement `Imu`, `Gps`, or `Lidar` Protocols in `src/hexapod_autonav/sensors/*`. Register usage in `sensors.yaml` and wire in `main.py`.

## Safety

-   E‑STOP flag halts the robot immediately.
-   Speed and turn rate limits are enforced via config.
-   Test in a controlled area before field tests.

## Limitations / TODO

-   Simple complementary yaw fusion; no full EKF/SLAM.
-   VFH-lite thresholds require tuning per platform/sensor.
-   Camera integration is stubbed for future work.

## License

See `LICENSE`.

## Contributing

PRs and issues welcome. Please run ruff and pytest locally.
