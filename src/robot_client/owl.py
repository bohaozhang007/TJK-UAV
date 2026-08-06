"""Agent-facing client for the VisBot OWL mini3L Robot Server."""

from __future__ import annotations

from pathlib import Path
from typing import Tuple

import numpy as np

from .base import JsonObject
from .tello import TelloClient


class OwlClient(TelloClient):
    """OWL HTTP client with client-side DA3 metric depth estimation.

    OWL and Tello use the same client-side RGB-to-depth pipeline. Motion is
    inherited from ``BaseClient``: XYZ is sent first and yaw separately. Until
    yaw control is validated, non-zero yaw requests fail explicitly on Server.
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
            for name in ("odom_ok", "rgb_ok", "control_ready")
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


__all__ = ["OwlClient"]
