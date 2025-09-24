from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class TelemetrySample:
    """Normalized telemetry sample captured at a single publish tick.

    Attributes
    ----------
    stamp:
        Monotonic timestamp (seconds) when the reading was taken; used to derive ``time_boot_ms``.
    lat / lon:
        Geographic coordinates in decimal degrees (WGS84) of the vehicle.
    alt_amsl_m:
        Altitude above mean sea level in metres; serialized as ``GLOBAL_POSITION_INT.alt``.
    rel_alt_m:
        Altitude relative to home in metres; serialized as ``GLOBAL_POSITION_INT.relative_alt``.
    vx_m_s / vy_m_s / vz_m_s:
        Body-frame velocities expressed in metres per second; fed into the position and attitude messages.
    roll_rad / pitch_rad / yaw_rad:
        Euler attitude angles in radians following the ENU convention; mapped to ``ATTITUDE``.
    fix_type:
        MAVLink GPS fix type code (e.g. 3 = 3D fix) for ``GPS_RAW_INT``.
    satellites_visible:
        Number of satellites tracked by the GNSS receiver.
    hdop / vdop:
        Horizontal and vertical dilution of precision values used for position accuracy fields.
    speed_accuracy_m_s:
        Estimated speed accuracy in metres per second.
    horiz_accuracy_m / vert_accuracy_m:
        Horizontal and vertical position accuracy (metres) forwarded to MAVLink accuracy fields.
    battery_voltage_v:
        Pack voltage in volts if available, otherwise ``None`` to omit.
    battery_current_a:
        Pack current draw in amperes if available.
    battery_remaining_pct:
        Remaining battery percentage (0-100) or ``None`` when not reported.
    """

    stamp: float
    lat: float
    lon: float
    alt_amsl_m: float
    rel_alt_m: float
    vx_m_s: float
    vy_m_s: float
    vz_m_s: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    fix_type: int = 3
    satellites_visible: int = 8
    hdop: float = 0.8
    vdop: float = 1.2
    speed_accuracy_m_s: float = 0.5
    horiz_accuracy_m: float = 0.5
    vert_accuracy_m: float = 0.8
    battery_voltage_v: float | None = None
    battery_current_a: float | None = None
    battery_remaining_pct: float | None = None
