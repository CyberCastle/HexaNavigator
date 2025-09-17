from __future__ import annotations

import time
from typing import Protocol

from ..core.types import ImuData


class Imu(Protocol):
    """IMU interface."""

    def start(self) -> None: ...

    def stop(self) -> None: ...

    def read(self) -> ImuData: ...


class DummyImu:
    """Simulated IMU producing a constant yaw rate."""

    def __init__(self) -> None:
        self._running = False
        self._yaw = 0.0

    def start(self) -> None:
        self._running = True

    def stop(self) -> None:
        self._running = False

    def read(self) -> ImuData:
        self._yaw = (self._yaw + 1.0) % 360.0
        return ImuData(
            stamp=time.time(),
            yaw_deg=self._yaw,
            pitch_deg=0.0,
            roll_deg=0.0,
            gyro_z_dps=1.0,
        )


class PiImu(DummyImu):
    """Placeholder for a real Raspberry Pi IMU adapter.

    TODO: integrate specific driver (e.g., ICM20948/MPU9250) and perform yaw stabilization
    via complementary filter using gyro + magnetometer.
    """

    pass
