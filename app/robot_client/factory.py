"""Select a Robot implementation without exposing hardware details to missions."""
from .base import Robot


def create_robot(config) -> Robot:
    if config.get('type') == 'owl_ego':
        from .owl_ego import OwlEgoClient
        return OwlEgoClient(config)
    raise ValueError(f"Unsupported Robot type: {config.get('type')!r}")
