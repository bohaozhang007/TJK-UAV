"""Agent-facing client for the I7 UAV Robot Server."""

from __future__ import annotations

import numpy as np

from .base import BaseClient, JsonObject
from .owl import OwlClient


class I7Client(OwlClient):
    """I7 HTTP client with Agent-side metric depth from DA3.

    I7 and OWL intentionally share the same RGB/DA3 capture path and the
    simultaneous ``/move_relative_xyz_yaw`` command.  Their Robot-side health
    contracts differ, so I7 performs checks for its MAVROS/EGO navigation
    chain instead of OWL's Captain/yaw link.
    """

    def start(self) -> JsonObject:
        # Check DA3 before BaseClient.start can initiate the blocking takeoff
        # sequence. For I7, takeoff must be requested before selecting OFFBOARD.
        depth_health = self._depth_service.health()
        result = BaseClient.start(self)
        health = self._health_state()
        missing = [
            name
            for name in (
                "odom_ok",
                "rgb_ok",
                "nav_ok",
                "planner_ok",
                "goal_link_ok",
                "control_ready",
            )
            if health.get(name) is not True
        ]
        if missing:
            raise RuntimeError(
                "I7 is not ready for Agent control: " + ", ".join(missing)
            )

        # Validate one actual K40T frame through DA3 before mission motion.
        frame_rgb, depth_cm = self.capture(include_depth=True)
        result["health"] = health
        result["depth_service"] = depth_health
        result["depth_warmup"] = {
            "rgb_shape": list(frame_rgb.shape),
            "depth_shape": list(depth_cm.shape),
            "depth_dtype": str(depth_cm.dtype),
            "depth_unit": "cm",
            "valid_ratio": float(
                np.mean(np.isfinite(depth_cm) & (depth_cm > 0))
            ),
        }
        return result


__all__ = ["I7Client"]
