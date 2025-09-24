from __future__ import annotations

import math
import time
from pathlib import Path

import pytest
from pymavlink import mavutil  # type: ignore[import]

from core.types import Waypoint  # type: ignore[import]
from mavlink import MavlinkServer, TelemetrySample  # type: ignore[import]

try:
    from src.mission.qgc_plan_loader import load_qgc_plan  # type: ignore[import]
except ImportError:  # pragma: no cover - support editable installs
    from mission.qgc_plan_loader import load_qgc_plan  # type: ignore[import]


class MavlinkGcsClient:
    """Thin MAVLink client that interacts with the server through real UDP frames."""

    def __init__(self, *, port: int, system_id: int, component_id: int) -> None:
        """Open a UDP connection to the MAVLink server as if we were a GCS."""
        self.port = port
        self.system_id = system_id
        self.component_id = component_id
        self._conn = mavutil.mavlink_connection(
            f"udpout:127.0.0.1:{port}",
            source_system=system_id,
            source_component=component_id,
        )

    @property
    def conn(self):
        return self._conn

    def close(self) -> None:
        """Close the underlying UDP socket, tolerating pymavlink variations."""
        try:
            self._conn.close()  # type: ignore[attr-defined]
        except Exception:  # pragma: no cover - defensive
            pass

    def send_heartbeat(self) -> None:
        """Send a heartbeat so the server learns the source system/component."""
        self._conn.mav.heartbeat_send(  # type: ignore[attr-defined]
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )

    def wait_for(self, message_type: str, timeout: float = 5.0):
        """Poll the socket for a given MAVLink message type within ``timeout`` seconds."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self._conn.recv_match(type=message_type, blocking=False)
            if msg is not None:
                return msg
            time.sleep(0.01)
        return None

    def send_mission_count(self, *, count: int) -> None:
        """Announce how many mission items will follow in the upload."""
        self._conn.mav.mission_count_send(  # type: ignore[attr-defined]
            target_system=1,
            target_component=1,
            count=count,
        )

    def send_mission_request_list(self) -> None:
        """Ask the server for the current mission (pull workflow)."""
        try:
            self._conn.mav.mission_request_list_send(  # type: ignore[attr-defined]
                target_system=1,
                target_component=1,
                mission_type=mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            )
        except TypeError:  # pragma: no cover - MAVLink 1 fallback
            self._conn.mav.mission_request_list_send(  # type: ignore[attr-defined]
                target_system=1,
                target_component=1,
            )

    def send_mission_request_int(self, *, seq: int) -> None:
        """Request an individual mission item during a download sequence."""
        try:
            self._conn.mav.mission_request_int_send(  # type: ignore[attr-defined]
                target_system=1,
                target_component=1,
                seq=seq,
                mission_type=mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            )
        except TypeError:  # pragma: no cover - MAVLink 1 fallback
            self._conn.mav.mission_request_int_send(  # type: ignore[attr-defined]
                target_system=1,
                target_component=1,
                seq=seq,
            )

    def send_mission_ack(self) -> None:
        """Acknowledge mission download completion."""
        try:
            self._conn.mav.mission_ack_send(  # type: ignore[attr-defined]
                target_system=1,
                target_component=1,
                type=mavutil.mavlink.MAV_MISSION_ACCEPTED,
                mission_type=mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            )
        except TypeError:  # pragma: no cover - MAVLink 1 fallback
            self._conn.mav.mission_ack_send(  # type: ignore[attr-defined]
                target_system=1,
                target_component=1,
                type=mavutil.mavlink.MAV_MISSION_ACCEPTED,
            )

    def send_mission_item(self, *, seq: int, waypoint: Waypoint) -> None:
        """Send a single mission item as an integer-based packet (as QGC does)."""
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
            z=float(waypoint.alt or 0.0),
        )

    def send_command_long(self, *, command: int, param1: float = 0.0) -> None:
        """Transmit a COMMAND_LONG message (e.g., arm/disarm) to the server."""
        self._conn.mav.command_long_send(  # type: ignore[attr-defined]
            target_system=1,
            target_component=1,
            command=command,
            confirmation=0,
            param1=param1,
            param2=0,
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0,
        )


@pytest.fixture()
def gcs_identity() -> tuple[int, int]:
    return 255, mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER


def test_mission_upload_roundtrip(gcs_identity) -> None:
    """Verify that the server can request and ingest a mission uploaded via UDP."""
    server = MavlinkServer(bind_host="127.0.0.1", bind_port=14660, telemetry_rate_hz=20.0)
    server.start()
    try:
        sys_id, comp_id = gcs_identity
        gcs = MavlinkGcsClient(port=14660, system_id=sys_id, component_id=comp_id)
        try:
            gcs.send_heartbeat()
            time.sleep(0.1)

            heartbeat = gcs.wait_for("HEARTBEAT", timeout=5.0)
            assert heartbeat is not None

            waypoints = [Waypoint(lat=10.0, lon=20.0, alt=5.0), Waypoint(lat=10.1, lon=20.1, alt=6.0)]

            server.request_mission_upload()
            req = gcs.wait_for("MISSION_REQUEST_LIST", timeout=5.0)
            assert req is not None

            gcs.send_mission_count(count=len(waypoints))

            for seq, wp in enumerate(waypoints):
                gcs.send_mission_item(seq=seq, waypoint=wp)
                if seq < len(waypoints) - 1:
                    follow_up = gcs.wait_for("MISSION_REQUEST_INT", timeout=5.0)
                    assert follow_up is not None

            ack = gcs.wait_for("MISSION_ACK", timeout=5.0)
            assert ack is not None

            mission = server.poll_mission_update(timeout=2.0)
            assert mission is not None
            assert len(mission) == 2
            assert math.isclose(mission[0].lat, waypoints[0].lat)
            assert math.isclose(mission[1].lon, waypoints[1].lon)
        finally:
            gcs.close()
    finally:
        server.stop()


def test_command_queue_and_telemetry(gcs_identity) -> None:
    """Ensure COMMAND_LONG messages are queued and telemetry is broadcast over UDP."""
    port = 14661
    server = MavlinkServer(bind_host="127.0.0.1", bind_port=port, telemetry_rate_hz=20.0)
    server.start()
    try:
        sys_id, comp_id = gcs_identity
        gcs = MavlinkGcsClient(port=port, system_id=sys_id, component_id=comp_id)
        try:
            gcs.send_heartbeat()
            time.sleep(0.1)

            assert gcs.wait_for("HEARTBEAT", timeout=5.0) is not None

            gcs.send_command_long(command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, param1=1)

            ack = gcs.wait_for("COMMAND_ACK", timeout=5.0)
            assert ack is not None

            queued = server.poll_command(timeout=2.0)
            assert queued is not None
            assert queued.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM

            sample = TelemetrySample(
                stamp=time.time(),
                lat=11.0,
                lon=22.0,
                alt_amsl_m=50.0,
                rel_alt_m=5.0,
                vx_m_s=1.0,
                vy_m_s=0.0,
                vz_m_s=0.0,
                roll_rad=0.0,
                pitch_rad=0.0,
                yaw_rad=0.1,
                fix_type=3,
                satellites_visible=8,
                hdop=0.9,
                vdop=1.2,
            )
            server.publish_telemetry(sample)

            pos = gcs.wait_for("GLOBAL_POSITION_INT", timeout=5.0)
            assert pos is not None
            assert pos.lat == int(sample.lat * 1e7)
            assert pos.lon == int(sample.lon * 1e7)

            attitude = gcs.wait_for("ATTITUDE", timeout=5.0)
            assert attitude is not None
            assert pytest.approx(attitude.yaw, rel=1e-3) == sample.yaw_rad
        finally:
            gcs.close()
    finally:
        server.stop()


def test_mission_download_from_plan_file(gcs_identity) -> None:  # noqa: D401 - simple flow
    """Verify the server serves missions loaded from a QGC .plan file."""

    plan_path = Path(__file__).with_name("test.plan")
    waypoints = load_qgc_plan(str(plan_path))
    assert waypoints, "Plan file should yield at least one waypoint"

    server = MavlinkServer(bind_host="127.0.0.1", bind_port=14662, telemetry_rate_hz=10.0)
    server.set_mission_waypoints(waypoints)
    server.start()
    try:
        sys_id, comp_id = gcs_identity
        gcs = MavlinkGcsClient(port=14662, system_id=sys_id, component_id=comp_id)
        try:
            gcs.send_heartbeat()
            time.sleep(0.1)

            gcs.send_mission_request_list()
            mission_count = gcs.wait_for("MISSION_COUNT", timeout=5.0)
            assert mission_count is not None
            assert mission_count.count == len(waypoints)

            received: list[tuple[float, float, float]] = []
            for seq in range(len(waypoints)):
                gcs.send_mission_request_int(seq=seq)
                msg = gcs.wait_for("MISSION_ITEM_INT", timeout=5.0)
                assert msg is not None
                lat_raw = getattr(msg, "x", None)
                lon_raw = getattr(msg, "y", None)
                alt_raw = getattr(msg, "z", None)
                assert lat_raw is not None and lon_raw is not None and alt_raw is not None
                received.append((float(lat_raw) / 1e7, float(lon_raw) / 1e7, float(alt_raw)))

            gcs.send_mission_ack()

            for wp, (lat, lon, alt) in zip(waypoints, received):
                assert math.isclose(lat, wp.lat, rel_tol=0, abs_tol=1e-7)
                assert math.isclose(lon, wp.lon, rel_tol=0, abs_tol=1e-7)
                if wp.alt is not None:
                    assert math.isclose(alt, wp.alt, rel_tol=0, abs_tol=1e-3)
        finally:
            gcs.close()
    finally:
        server.stop()


def test_mission_upload_from_plan_file(gcs_identity) -> None:
    """Upload a mission sourced from a QGC .plan file and verify the server stores it."""

    plan_path = Path(__file__).with_name("test.plan")
    waypoints = load_qgc_plan(str(plan_path))
    assert waypoints, "Plan file should yield at least one waypoint"

    server = MavlinkServer(bind_host="127.0.0.1", bind_port=14663, telemetry_rate_hz=10.0)
    server.start()
    try:
        sys_id, comp_id = gcs_identity
        gcs = MavlinkGcsClient(port=14663, system_id=sys_id, component_id=comp_id)
        try:
            gcs.send_heartbeat()
            time.sleep(0.1)

            server.request_mission_upload()
            req = gcs.wait_for("MISSION_REQUEST_LIST", timeout=5.0)
            assert req is not None

            gcs.send_mission_count(count=len(waypoints))

            for seq, wp in enumerate(waypoints):
                gcs.send_mission_item(seq=seq, waypoint=wp)
                if seq < len(waypoints) - 1:
                    follow_up = gcs.wait_for("MISSION_REQUEST_INT", timeout=5.0)
                    assert follow_up is not None

            ack = gcs.wait_for("MISSION_ACK", timeout=5.0)
            assert ack is not None

            uploaded = server.poll_mission_update(timeout=2.0)
            assert uploaded is not None
            assert len(uploaded) == len(waypoints)

            for original, stored in zip(waypoints, uploaded):
                assert math.isclose(original.lat, stored.lat, rel_tol=0, abs_tol=1e-7)
                assert math.isclose(original.lon, stored.lon, rel_tol=0, abs_tol=1e-7)
                if original.alt is not None and stored.alt is not None:
                    assert math.isclose(original.alt, stored.alt, rel_tol=0, abs_tol=1e-3)
        finally:
            gcs.close()
    finally:
        server.stop()
