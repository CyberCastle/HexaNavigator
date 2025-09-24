# Hexapod AutoNav

Autonomous waypoint navigation for a Raspberry Pi–mounted hexapod robot using QGroundControl missions, IMU, GPS/RTK, and LiDAR. No move_to(x,y) is assumed; the controller translates plans into directional RPC commands (forward/backward/turn/stop).

-   Mission input: QGC .plan (JSON) and MAVLink plain text .waypoints (optional)
-   Sensors: IMU, GPS/RTK, LiDAR (camera optional)
-   Planner: Pure Pursuit + reactive obstacle avoidance (sectorization/VFH-lite)
-   Control: UART-based RPC (library TBD) for hexapod directional API
-   State machine: IDLE → NAVIGATING ↔ AVOIDING → COMPLETED/ABORTED/PAUSED

## MAVLink integration

The project ships with a lightweight MAVLink 2 UDP server (`src/mavlink/server.py`) built on top of
`pymavlink`. It allows QGroundControl (or any GCS) to push missions, issue high-level commands, and
subscribe to live telemetry while the pure-pursuit navigator runs locally.

Key capabilities

-   Listens on `0.0.0.0:14550` by default (override via `config/default.yaml` → `mavlink` section).
-   Accepts `MISSION_COUNT`/`MISSION_ITEM_INT` uploads and forwards them to the navigator in real time.
-   Sends `MISSION_REQUEST[_INT]` and `MISSION_COUNT` replies when the GCS pulls the current plan.
-   Queues `COMMAND_LONG` messages (e.g. arm/disarm, mission start, RTL) and routes them to the state
    machine with ACKs.
-   Publishes telemetry at the configured rate using `GLOBAL_POSITION_INT`, `ATTITUDE`, `GPS_RAW_INT`,
    and `SYS_STATUS`, populated from the cached sensor readings in the navigator (`TelemetrySample`).
-   Speaks MAVLink 2 by default, with graceful fallback to MAVLink 1 if a client lacks newer
    extensions.

Configuration

```yaml
mavlink:
		enabled: true
		bind_host: "0.0.0.0"
		bind_port: 14550
		system_id: 1
		component_id: 1
		heartbeat_rate_hz: 1.0
		telemetry_rate_hz: 2.0
		auto_request_mission: true
```

You can disable the server (`enabled: false`), change the bind address/port, or adjust heartbeat and
telemetry rates without modifying code. Command-line overrides can be added later if needed.

Testing

The suite includes integration tests that drive the server using a real UDP client based on
`pymavlink`:

```bash
poetry run pytest tests/test_mavlink_server.py -q
```

These tests validate mission upload handshakes, command acknowledgements, and telemetry broadcast
end to end.

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

Default (dummy sensors), run the app module:

```bash
python -m src.main run --plan mission.plan --speed 0.15 --gait tripod
```

Or install the package in editable mode and run the same command.

Configuration:

-   Edit `src/config/default.yaml` for navigation parameters.
-   Edit `src/config/sensors.yaml` to select adapters (dummy/real) and ports.

## RPC integration (UART)

RPC will be accessed over UART via a serial library (TBD). The concrete adapter/driver is not defined yet; a `RobotAPI` implementation for UART should be added and wired in when the library is chosen.

-   Configure serial port and baud in `config/sensors.yaml` (to be extended) and/or CLI flags when implemented.
-   Implement a `UartRobotAPI` alongside the existing interface in `src/robot/` and use it within the controller.

Note: Previous HTTP/JSON-RPC details have been removed in favor of the UART approach.

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
