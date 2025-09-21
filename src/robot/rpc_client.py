from __future__ import annotations

import json
import logging
from dataclasses import dataclass
from typing import Optional, Protocol

from .api_interface import RobotAPI

log = logging.getLogger(__name__)


class ByteWriter(Protocol):
    """Minimal protocol for a byte-oriented writer (e.g., pyserial Serial)."""

    def write(self, data: bytes) -> int: ...
    def flush(self) -> None: ...


@dataclass
class UartRobotAPI(RobotAPI):
    """
    UART-based implementation of RobotAPI.

    Notes:
    - The concrete serial library is TBD. If `pyserial` is present, it will be used.
    - Otherwise, this acts as a no-op stub that logs the intended frames.
    - Frames are JSON lines (UTF-8) like: {"cmd": "forward", "speed": 0.15}\n
    """

    port: Optional[str] = None
    baudrate: int = 115200
    timeout_s: float = 0.2
    writer: Optional[ByteWriter] = None

    def __post_init__(self) -> None:
        self._ser = None
        if self.writer is None and self.port:
            serial_mod = None
            try:  # soft import pyserial if available
                import serial as serial_mod  # type: ignore
            except Exception:  # pragma: no cover - optional dependency
                serial_mod = None

            if serial_mod is not None:
                try:
                    self._ser = serial_mod.Serial(self.port, self.baudrate, timeout=self.timeout_s)

                    class _SerialWriter:
                        def __init__(self, ser):
                            self._ser = ser

                        def write(self, data: bytes) -> int:
                            r = self._ser.write(data)
                            return 0 if r is None else int(r)

                        def flush(self) -> None:
                            try:
                                self._ser.flush()
                            except Exception:
                                pass

                    self.writer = _SerialWriter(self._ser)
                    log.info("UART opened on %s @ %d bps", self.port, self.baudrate)
                except Exception as e:  # pragma: no cover - hardware dependent
                    log.warning("Failed to open UART on %s: %s", self.port, e)
            else:
                log.info("pyserial not installed; UART disabled (stub mode)")

    def _send(self, cmd: str, payload: dict | None = None) -> None:
        msg = {"cmd": cmd}
        if payload:
            msg.update(payload)
        data = (json.dumps(msg, separators=(",", ":")) + "\n").encode("utf-8")
        if self.writer is not None:
            try:
                self.writer.write(data)
                try:
                    self.writer.flush()
                except Exception:  # some writers may not implement flush
                    pass
            except Exception as e:
                log.warning("UART send failed: %s", e)
        else:
            log.debug("UART stub frame: %s", msg)

    # RobotAPI methods
    def forward(self, speed: float) -> None:
        self._send("forward", {"speed": float(speed)})

    def backward(self, speed: float) -> None:
        self._send("backward", {"speed": float(speed)})

    def turn_left(self, rate: float) -> None:
        self._send("turn_left", {"rate": float(rate)})

    def turn_right(self, rate: float) -> None:
        self._send("turn_right", {"rate": float(rate)})

    def strafe_left(self, speed: float) -> None:
        self._send("strafe_left", {"speed": float(speed)})

    def strafe_right(self, speed: float) -> None:
        self._send("strafe_right", {"speed": float(speed)})

    def stop(self) -> None:
        self._send("stop", {})

    def set_velocity(self, v: float) -> None:
        self._send("set_velocity", {"v": float(v)})

    def set_gait(self, name: str) -> None:
        self._send("set_gait", {"name": name})

    def set_pose(self, **kwargs) -> None:
        # Pass-through of arbitrary pose kwargs
        self._send("set_pose", {k: v for k, v in kwargs.items()})


__all__ = ["UartRobotAPI", "ByteWriter"]
