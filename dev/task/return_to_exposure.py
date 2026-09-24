def run(flight, exposure_pose):
    """Acquire control and return to the saved exposure position and yaw."""
    return flight.go_to_pose(exposure_pose)
