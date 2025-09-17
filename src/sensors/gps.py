from __future__ import annotations

import time
from typing import Protocol, Optional

from ..core.types import GpsData


class Gps(Protocol):
    def read(self) -> GpsData: ...

    def has_fix(self) -> bool: ...


class DummyGps:
    def __init__(self) -> None:
        self._lat = 0.0
        self._lon = 0.0
        self._alt = 0.0

    def read(self) -> GpsData:
        return GpsData(
            stamp=time.time(),
            lat=self._lat,
            lon=self._lon,
            alt=self._alt,
            fix_type=3,
            rtk=False,
            speed_mps=0.0,
        )

    def has_fix(self) -> bool:
        return True


class SerialNmeaGps(DummyGps):
    """Serial NMEA GPS adapter using pyserial+pynmea2.

    TODO: implement actual serial reading and NMEA parsing.
    """

    def __init__(self, port: str, baudrate: int = 115200) -> None:
        super().__init__()
        self._port = port
        self._baud = baudrate
        # TODO: open serial and parse NMEA
