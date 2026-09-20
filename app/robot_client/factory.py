"""Select the local Robot adapter from its advertised capabilities."""
from .base import Robot, MissionError
from .owl_ego import OwlEgoClient


def create_robot(localization, *, autofocus=False, tracker_host=None) -> Robot:
    probe = OwlEgoClient(**localization)
    backend = probe.rpc('GET', '/v21/capabilities').get('backend')
    if backend == 'i7':
        from .i7 import I7Client
        robot = I7Client(**localization)
    elif backend == 'owl_ego':
        robot = probe
    else:
        raise MissionError('unsupported Robot backend: ' + str(backend))
    robot.autofocus_enabled = autofocus
    robot.tracker_host = tracker_host
    return robot
