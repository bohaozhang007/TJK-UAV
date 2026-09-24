def run(flight, exposure_pose):
    """Restore the exposure pose, then release control before detection restarts."""
    if not flight.go_to_pose(exposure_pose):
        return False
    return flight.release_control()
