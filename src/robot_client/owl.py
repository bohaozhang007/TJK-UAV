"""Agent-facing client for the VisBot OWL mini3L Robot Server."""

from __future__ import annotations

from pathlib import Path
from typing import Tuple

import numpy as np

from .base import JsonObject
from .tello import TelloClient


class OwlClient(TelloClient):
    """OWL HTTP client with client-side DA3 metric depth estimation.

    OWL and Tello use the same client-side RGB-to-depth pipeline. OWL motion
    uses the Server's simultaneous XYZ/yaw endpoint.
    """

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 8765,
        timeout_s: float = 180.0,
        *,
        depth_model_dir: str | Path | None = None,
    ) -> None:
        super().__init__(
            host=host,
            port=port,
            timeout_s=timeout_s,
            depth_model_dir=depth_model_dir,
        )

    def start(self) -> JsonObject:
        result = super().start()
        health = self._health_state()
        missing = [
            name
            for name in (
                "odom_ok",
                "rgb_ok",
                "yaw_link_ok",
                "control_ready",
            )
            if health.get(name) is not True
        ]
        if missing:
            raise RuntimeError(
                "OWL is not ready for Agent control: " + ", ".join(missing)
            )
        result["health"] = health
        return result

    def capture(
        self,
        include_depth: bool = True,
        raw: bool = True,
    ) -> np.ndarray | Tuple[np.ndarray, np.ndarray]:
        return super().capture(include_depth=include_depth, raw=raw)

    def move_relative(
        self,
        dx: float = 0.0,
        dy: float = 0.0,
        dz: float = 0.0,
        dyaw: float = 0.0,
    ) -> JsonObject:
        x, y, z = (int(round(value)) for value in (dx, dy, dz))
        yaw = int(round(dyaw))
        if not any((x, y, z, yaw)):
            return {
                "ok": True,
                "message": "move_relative_xyz_yaw skipped",
                "command": {"x": x, "y": y, "z": z, "yaw": yaw},
            }
        return self._require_ok(
            self._request_json(
                "POST",
                "/move_relative_xyz_yaw",
                {"x": x, "y": y, "z": z, "yaw": yaw},
            ),
            "move_relative_xyz_yaw",
        )


__all__ = ["OwlClient"]
