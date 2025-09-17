from __future__ import annotations

import logging
from dataclasses import dataclass

import requests

from .api_interface import RobotAPI

log = logging.getLogger(__name__)


@dataclass
class HttpRpcRobotAPI(RobotAPI):
    base_url: str
    timeout_s: float = 2.0

    def _post(self, path: str, payload: dict | None = None) -> None:
        url = self.base_url.rstrip("/") + path
        try:
            r = requests.post(url, json=payload or {}, timeout=self.timeout_s)
            if r.status_code >= 400:
                log.warning("RPC error %s: %s", r.status_code, r.text)
        except requests.RequestException as e:
            log.warning("RPC request failed: %s", e)

    def forward(self, speed: float) -> None:
        self._post("/cmd/forward", {"speed": float(speed)})

    def backward(self, speed: float) -> None:
        self._post("/cmd/backward", {"speed": float(speed)})

    def turn_left(self, rate: float) -> None:
        self._post("/cmd/turn_left", {"rate": float(rate)})

    def turn_right(self, rate: float) -> None:
        self._post("/cmd/turn_right", {"rate": float(rate)})

    def stop(self) -> None:
        self._post("/cmd/stop", {})

    def set_velocity(self, v: float) -> None:
        self._post("/config/set_velocity", {"v": float(v)})

    def set_gait(self, name: str) -> None:
        self._post("/config/set_gait", {"name": name})

    def set_pose(self, **kwargs) -> None:
        self._post("/config/set_pose", kwargs)

    def strafe_left(self, speed: float) -> None:
        # Not all hexapods support strafing; raise if endpoint is missing.
        self._post("/cmd/strafe_left", {"speed": float(speed)})

    def strafe_right(self, speed: float) -> None:
        self._post("/cmd/strafe_right", {"speed": float(speed)})
