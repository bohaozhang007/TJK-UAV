"""Construct the current Robot adapter without exposing hardware choices to missions."""
from .base import Robot


def create_robot(localization) -> Robot:
    from .owl_ego import OwlEgoClient
    return OwlEgoClient(**localization)
