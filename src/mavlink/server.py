from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass
from queue import Empty, Queue
from typing import Any, Iterable, Optional, Sequence

from pymavlink import mavutil  # type: ignore[import]

try:
    from ..core.types import Waypoint
except ImportError:  # pragma: no cover - support when package imported as top-level
    from core.types import Waypoint  # type: ignore[import]

from .telemetry import TelemetrySample

log = logging.getLogger(__name__)


@dataclass(slots=True)
class CommandMessage:
    """MAVLink COMMAND_LONG request captured for higher-level handling."""

    command: int
    params: tuple[float, float, float, float, float, float, float]
    target_system: int
    target_component: int
    confirmation: int
    source_system: int
    source_component: int


class MavlinkServer:
    """Light-weight MAVLink UDP server exposing mission, command, and telemetry plumbing."""

    def __init__(
        self,
        *,
        bind_host: str = "0.0.0.0",
        bind_port: int = 14550,
        system_id: int = 1,
        component_id: int = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
        vehicle_type: int = mavutil.mavlink.MAV_TYPE_GROUND_ROVER,
        autopilot_type: int = mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
        heartbeat_rate_hz: float = 1.0,
        telemetry_rate_hz: float = 2.0,
    ) -> None:
        self.bind_host = bind_host
        self.bind_port = int(bind_port)
        self.system_id = int(system_id)
        self.component_id = int(component_id)
        self.vehicle_type = vehicle_type
        self.autopilot_type = autopilot_type
        self.heartbeat_period = 1.0 / max(heartbeat_rate_hz, 0.1)
        self.telemetry_period = 1.0 / max(telemetry_rate_hz, 0.1)

        self._conn: Optional[mavutil.mavfile] = None
        self._read_thread: Optional[threading.Thread] = None
        self._io_thread: Optional[threading.Thread] = None
        self._running = False
        self._boot_time = time.time()
        self._connection_ready = threading.Event()

        self._mission_store: list[Waypoint] = []
        self._mission_lock = threading.Lock()
        self._mission_updates: Queue[list[Waypoint]] = Queue()

        self._command_queue: Queue[CommandMessage] = Queue()
        self._last_telemetry: Optional[TelemetrySample] = None
        self._telemetry_lock = threading.Lock()

        self._mission_upload_expected: Optional[int] = None
        self._mission_upload_items: dict[int, Any] = {}
        self._mission_upload_type: int = mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        self._mission_current_idx: int = 0

        self._last_gcs_system: Optional[int] = None
        self._last_gcs_component: Optional[int] = None

        self.base_mode = mavutil.mavlink.MAV_MODE_MANUAL_ARMED
        self.custom_mode = 0
        self.system_status = mavutil.mavlink.MAV_STATE_ACTIVE

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def start(self) -> None:
        if self._running:
            return

        url = f"udp:{self.bind_host}:{self.bind_port}"
        log.info("Starting MAVLink server on %s", url)
        self._conn = mavutil.mavlink_connection(
            url,
            source_system=self.system_id,
            source_component=self.component_id,
            autoreconnect=True,
        )
        self._mav.srcSystem = self.system_id
        self._mav.srcComponent = self.component_id

        self._running = True
        self._read_thread = threading.Thread(target=self._read_loop, name="mavlink-read", daemon=True)
        self._io_thread = threading.Thread(target=self._io_loop, name="mavlink-io", daemon=True)
        self._read_thread.start()
        self._io_thread.start()
        self._connection_ready.set()

    def stop(self) -> None:
        if not self._running:
            return
        log.info("Stopping MAVLink server")
        self._running = False
        self._connection_ready.clear()
        if self._read_thread and self._read_thread.is_alive():
            self._read_thread.join(timeout=2.0)
        if self._io_thread and self._io_thread.is_alive():
            self._io_thread.join(timeout=2.0)
        if self._conn is not None:
            try:
                self._conn.close()
            except Exception:  # pragma: no cover - defensive
                pass
        self._conn = None

    # ------------------------------------------------------------------
    # External interfaces
    # ------------------------------------------------------------------
    def publish_telemetry(self, sample: TelemetrySample) -> None:
        with self._telemetry_lock:
            self._last_telemetry = sample

    def poll_mission_update(self, timeout: float | None = None) -> Optional[list[Waypoint]]:
        try:
            if timeout is None or timeout <= 0:
                return self._mission_updates.get_nowait()
            return self._mission_updates.get(timeout=timeout)
        except Empty:
            return None

    def poll_command(self, timeout: float | None = None) -> Optional[CommandMessage]:
        try:
            if timeout is None or timeout <= 0:
                return self._command_queue.get_nowait()
            return self._command_queue.get(timeout=timeout)
        except Empty:
            return None

    def set_mission_waypoints(self, waypoints: Sequence[Waypoint]) -> None:
        cloned = [wp.model_copy(deep=True) for wp in waypoints]
        with self._mission_lock:
            self._mission_store = cloned
            self._mission_current_idx = 0

    def get_mission_waypoints(self) -> list[Waypoint]:
        with self._mission_lock:
            return [wp.model_copy(deep=True) for wp in self._mission_store]

    def request_mission_upload(self, mission_type: int = mavutil.mavlink.MAV_MISSION_TYPE_MISSION) -> None:
        if not self._connection_ready.is_set() or self._conn is None:
            return
        target_system = self._last_gcs_system or 0
        target_component = self._last_gcs_component or mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER
        log.info(
            "Requesting mission upload (target_system=%s, target_component=%s, type=%s)",
            target_system,
            target_component,
            mission_type,
        )
        self._safe_send(
            self._mav.mission_request_list_send,
            target_system=target_system,
            target_component=target_component,
            mission_type=mission_type,
        )

    # ------------------------------------------------------------------
    # Internal loops
    # ------------------------------------------------------------------
    def _read_loop(self) -> None:
        assert self._conn is not None
        while self._running:
            try:
                msg = self._conn.recv_match(blocking=True, timeout=0.5)
            except Exception as exc:  # pragma: no cover - transport error
                log.debug("MAVLink recv error: %s", exc)
                continue
            if not self._running:
                break
            if msg is None:
                continue
            msg_type = msg.get_type()
            self._last_gcs_system = getattr(msg, "get_srcSystem", lambda: None)() or self._last_gcs_system
            self._last_gcs_component = getattr(msg, "get_srcComponent", lambda: None)() or self._last_gcs_component
            handler_name = f"_handle_{msg_type.lower()}"
            handler = getattr(self, handler_name, None)
            if handler:
                try:
                    handler(msg)
                except Exception as exc:  # pragma: no cover - defensive
                    log.warning("MAVLink handler %s failed: %s", handler_name, exc)

    def _io_loop(self) -> None:
        mav = self._mav
        last_hb = 0.0
        last_telemetry = 0.0
        while self._running:
            now = time.time()
            if now - last_hb >= self.heartbeat_period:
                self._send_heartbeat(mav)
                last_hb = now
            send_tel = False
            with self._telemetry_lock:
                sample = self._last_telemetry
            if sample and now - last_telemetry >= self.telemetry_period:
                send_tel = True
                last_telemetry = now
            if send_tel and sample:
                self._send_telemetry(mav, sample)
            time.sleep(0.05)

    # ------------------------------------------------------------------
    # Message handlers
    # ------------------------------------------------------------------
    def _handle_heartbeat(self, msg) -> None:
        # Nothing special yet; store last sender via _last_gcs_* updates
        log.debug(
            "Heartbeat received from system=%s component=%s",
            msg.get_srcSystem(),
            msg.get_srcComponent(),
        )

    def _handle_mission_request_list(self, msg) -> None:
        if not self._is_targeted(msg):
            return
        mission_type = getattr(msg, "mission_type", mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        with self._mission_lock:
            count = len(self._mission_store)
        log.info(
            "GCS requested mission list (count=%d, mission_type=%d)",
            count,
            mission_type,
        )
        self._safe_send(
            self._mav.mission_count_send,
            target_system=msg.get_srcSystem(),
            target_component=msg.get_srcComponent(),
            count=count,
            mission_type=mission_type,
        )

    def _handle_mission_request(self, msg) -> None:
        self._handle_mission_request_int(msg)

    def _handle_mission_request_int(self, msg) -> None:
        if not self._is_targeted(msg):
            return
        seq = int(msg.seq)
        mission_type = getattr(msg, "mission_type", mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        self._send_mission_item(seq, mission_type, msg.get_srcSystem(), msg.get_srcComponent())

    def _handle_mission_count(self, msg) -> None:
        if not self._is_targeted(msg):
            return
        count = int(msg.count)
        self._mission_upload_expected = count
        self._mission_upload_items = {}
        self._mission_upload_type = getattr(msg, "mission_type", mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        log.info("Mission upload initiated (expected %d items)", count)
        if count == 0:
            self._finalize_mission_upload()
            return
        self._request_next_mission_item(seq=0)

    def _handle_mission_item(self, msg) -> None:
        # Convert float lat/lon to int representation-like for reuse
        msg_int = self._mav.mission_item_int_encode(
            target_system=msg.target_system,
            target_component=msg.target_component,
            seq=msg.seq,
            frame=msg.frame,
            command=msg.command,
            current=msg.current,
            autocontinue=msg.autocontinue,
            param1=msg.param1,
            param2=msg.param2,
            param3=msg.param3,
            param4=msg.param4,
            x=int(msg.x * 1e7),
            y=int(msg.y * 1e7),
            z=msg.z,
            mission_type=getattr(msg, "mission_type", mavutil.mavlink.MAV_MISSION_TYPE_MISSION),
        )
        self._handle_mission_item_int(msg_int)

    def _handle_mission_item_int(self, msg) -> None:
        if not self._is_targeted(msg):
            return
        if self._mission_upload_expected is None:
            log.debug("Received unexpected mission item (seq=%d)", msg.seq)
            return
        self._mission_upload_items[int(msg.seq)] = msg
        if len(self._mission_upload_items) < self._mission_upload_expected:
            next_seq = int(msg.seq) + 1
            self._request_next_mission_item(next_seq)
        else:
            self._finalize_mission_upload()

    def _handle_mission_clear_all(self, msg) -> None:
        if not self._is_targeted(msg):
            return
        log.info("Received MISSION_CLEAR_ALL from GCS")
        self.set_mission_waypoints([])
        self._mission_updates.put([])
        self._send_mission_ack(
            mavutil.mavlink.MAV_MISSION_ACCEPTED,
            msg.get_srcSystem(),
            msg.get_srcComponent(),
            getattr(msg, "mission_type", mavutil.mavlink.MAV_MISSION_TYPE_MISSION),
        )

    def _handle_command_long(self, msg) -> None:
        if not self._is_targeted(msg):
            return
        params = (
            float(msg.param1),
            float(msg.param2),
            float(msg.param3),
            float(msg.param4),
            float(msg.param5),
            float(msg.param6),
            float(msg.param7),
        )
        command = CommandMessage(
            command=int(msg.command),
            params=params,
            target_system=int(msg.target_system),
            target_component=int(msg.target_component),
            confirmation=int(getattr(msg, "confirmation", 0)),
            source_system=int(msg.get_srcSystem()),
            source_component=int(msg.get_srcComponent()),
        )
        self._command_queue.put(command)
        log.info("COMMAND_LONG received: command=%d params=%s", command.command, params)
        self._safe_send(
            self._mav.command_ack_send,
            command.command,
            mavutil.mavlink.MAV_RESULT_ACCEPTED,
        )

    def _handle_mission_ack(self, msg) -> None:
        log.info(
            "MISSION_ACK received from system=%s type=%s",
            msg.get_srcSystem(),
            msg.type,
        )

    # ------------------------------------------------------------------
    # Mission helpers
    # ------------------------------------------------------------------
    def _request_next_mission_item(self, seq: int) -> None:
        target_system = self._last_gcs_system or 0
        target_component = self._last_gcs_component or mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER
        self._safe_send(
            self._mav.mission_request_int_send,
            target_system=target_system,
            target_component=target_component,
            seq=seq,
            mission_type=self._mission_upload_type,
        )

    def _finalize_mission_upload(self) -> None:
        expected = self._mission_upload_expected or 0
        waypoints: list[Waypoint] = []
        for idx in range(expected):
            msg = self._mission_upload_items.get(idx)
            if msg is None:
                log.warning("Mission upload missing item %d", idx)
                return
            lat = float(msg.x) / 1e7
            lon = float(msg.y) / 1e7
            alt = float(msg.z)
            tolerance = float(msg.param2) if not math.isnan(msg.param2) else 1.0
            hold = float(msg.param1) if not math.isnan(msg.param1) else 0.0
            heading = float(msg.param4) if not math.isnan(msg.param4) else None
            wp = Waypoint(
                lat=lat,
                lon=lon,
                alt=alt,
                tolerance_m=tolerance if tolerance > 0 else 1.0,
                hold_s=hold if hold >= 0 else 0.0,
                heading_deg=heading,
            )
            waypoints.append(wp)
        self.set_mission_waypoints(waypoints)
        if waypoints:
            log.info("Mission upload completed (%d waypoints)", len(waypoints))
        else:
            log.info("Mission cleared via upload")
        self._mission_updates.put([wp.model_copy(deep=True) for wp in waypoints])
        self._send_mission_ack(
            mavutil.mavlink.MAV_MISSION_ACCEPTED,
            self._last_gcs_system or 0,
            self._last_gcs_component or mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
            self._mission_upload_type,
        )
        self._mission_upload_expected = None
        self._mission_upload_items = {}

    def _send_mission_item(
        self,
        seq: int,
        mission_type: int,
        target_system: int,
        target_component: int,
    ) -> None:
        with self._mission_lock:
            if seq >= len(self._mission_store):
                log.warning("Mission request out of range (seq=%d, mission_size=%d)", seq, len(self._mission_store))
                return
            wp = self._mission_store[seq]
        current = 1 if seq == self._mission_current_idx else 0
        lat_i = int(wp.lat * 1e7)
        lon_i = int(wp.lon * 1e7)
        alt = float(wp.alt or 0.0)
        heading = wp.heading_deg if wp.heading_deg is not None else 0.0
        self._safe_send(
            self._mav.mission_item_int_send,
            target_system=target_system,
            target_component=target_component,
            seq=seq,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            current=current,
            autocontinue=1,
            param1=wp.hold_s,
            param2=wp.tolerance_m,
            param3=0.0,
            param4=heading,
            x=lat_i,
            y=lon_i,
            z=alt,
            mission_type=mission_type,
        )

    def _send_mission_ack(self, ack_type: int, target_system: int, target_component: int, mission_type: int) -> None:
        self._safe_send(
            self._mav.mission_ack_send,
            target_system=target_system,
            target_component=target_component,
            type=ack_type,
            mission_type=mission_type,
        )

    # ------------------------------------------------------------------
    # Low level send helpers
    # ------------------------------------------------------------------
    @property
    def _mav(self):
        conn = self._conn
        if conn is None:
            raise RuntimeError("MAVLink connection is not active")
        return conn.mav

    def _send_heartbeat(self, mav) -> None:
        self._safe_send(
            mav.heartbeat_send,
            type=self.vehicle_type,
            autopilot=self.autopilot_type,
            base_mode=self.base_mode,
            custom_mode=self.custom_mode,
            system_status=self.system_status,
        )

    def _send_telemetry(self, mav, sample: TelemetrySample) -> None:
        time_boot_ms = int(max(0.0, sample.stamp - self._boot_time) * 1000)
        lat = int(sample.lat * 1e7)
        lon = int(sample.lon * 1e7)
        alt_mm = int(sample.alt_amsl_m * 1000)
        rel_alt_mm = int(sample.rel_alt_m * 1000)
        vx = int(sample.vx_m_s * 100)
        vy = int(sample.vy_m_s * 100)
        vz = int(sample.vz_m_s * 100)
        heading_cdeg = int((math.degrees(sample.yaw_rad) % 360.0) * 100)
        cog_cdeg = heading_cdeg
        vel_3d = int(max(0.0, math.sqrt(sample.vx_m_s**2 + sample.vy_m_s**2 + sample.vz_m_s**2)) * 100)
        eph = int(sample.hdop * 100)
        epv = int(sample.vdop * 100)
        h_acc = int(sample.horiz_accuracy_m * 1000)
        v_acc = int(sample.vert_accuracy_m * 1000)
        vel_acc = int(sample.speed_accuracy_m_s * 100)
        voltage_mv = int(sample.battery_voltage_v * 1000) if sample.battery_voltage_v is not None else 0
        current_ca = int(sample.battery_current_a * 100) if sample.battery_current_a is not None else 0
        battery_remaining = (
            int(sample.battery_remaining_pct)
            if sample.battery_remaining_pct is not None
            else mavutil.mavlink.MAV_BATTERY_CHARGE_STATE_UNDEFINED
        )

        self._safe_send(
            mav.global_position_int_send,
            time_boot_ms,
            lat,
            lon,
            alt_mm,
            rel_alt_mm,
            vx,
            vy,
            vz,
            heading_cdeg,
        )
        self._safe_send(
            mav.attitude_send,
            time_boot_ms,
            sample.roll_rad,
            sample.pitch_rad,
            sample.yaw_rad,
            0.0,
            0.0,
            0.0,
        )
        self._safe_send(
            mav.gps_raw_int_send,
            int(sample.stamp * 1e6),
            sample.fix_type,
            lat,
            lon,
            alt_mm,
            eph,
            epv,
            vel_3d,
            cog_cdeg,
            sample.satellites_visible,
            alt_mm,
            h_acc,
            v_acc,
            vel_acc,
            0,
            heading_cdeg,
        )
        self._safe_send(
            mav.sys_status_send,
            0,
            0,
            0,
            0,
            voltage_mv,
            current_ca,
            battery_remaining,
            0,
            0,
            0,
            0,
            0,
            0,
        )

    def _safe_send(self, func, *args, **kwargs) -> None:
        if self._conn is None:
            return
        kwargs_local = dict(kwargs)
        try:
            func(*args, **kwargs_local)
        except TypeError as exc:
            if "mission_type" in kwargs_local:
                kwargs_fallback = dict(kwargs_local)
                kwargs_fallback.pop("mission_type", None)
                try:
                    func(*args, **kwargs_fallback)
                    return
                except Exception as inner_exc:  # pragma: no cover - fallback error
                    log.debug(
                        "Failed to send MAVLink message %s without mission_type: %s",
                        func.__name__,
                        inner_exc,
                    )
                    return
            log.debug("Failed to send MAVLink message %s: %s", func.__name__, exc)
        except Exception as exc:  # pragma: no cover - transport error
            log.debug("Failed to send MAVLink message %s: %s", func.__name__, exc)

    # ------------------------------------------------------------------
    # Utilities
    # ------------------------------------------------------------------
    def _is_targeted(self, msg) -> bool:
        target_system = int(getattr(msg, "target_system", self.system_id))
        return target_system in (self.system_id, 0)


__all__ = ["MavlinkServer", "TelemetrySample", "CommandMessage"]
