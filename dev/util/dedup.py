import numpy as np


DEDUP_DISTANCE_M = 1.8


def select_new_targets(detections, seen_positions):
    """Filter observed targets and batch duplicates without recording observations."""
    selected = []
    batch_positions = {frame: list(positions) for frame, positions in seen_positions.items()}
    for detection in detections:
        if detection["position_world_m"] is None:
            continue
        position = np.asarray(detection["position_world_m"], dtype=float)
        if (
            position.shape != (3,)
            or not np.isfinite(position).all()
        ):
            continue
        positions = batch_positions.setdefault(detection["world_frame"], [])
        if any(np.all(np.abs(position - old) < DEDUP_DISTANCE_M) for old in positions):
            continue
        positions.append(position.copy())
        selected.append(detection)
    return selected
