import numpy as np


DEDUP_DISTANCE_M = 1.0


def select_new_targets(detections, seen_positions):
    """Keep new localized targets using v22's per-axis distance rule."""
    selected = []
    for detection in detections:
        if detection["position_world_m"] is None:
            continue
        position = np.asarray(detection["position_world_m"], dtype=float)
        if (
            position.shape != (3,)
            or not np.isfinite(position).all()
        ):
            continue
        positions = seen_positions.setdefault(detection["world_frame"], [])
        if any(np.all(np.abs(position - old) < DEDUP_DISTANCE_M) for old in positions):
            continue
        positions.append(position.copy())
        selected.append(detection)
    return selected
