"""Physical Tello hardware dependency exposed through the hardware layer."""

from djitellopy import Tello

MOVE_SPEED = 30

__all__ = ["MOVE_SPEED", "Tello"]
