"""Small, JSON-safe localization diagnostics; never include image arrays."""

import math

import numpy as np

from util.timing import log_timing


def _safe(value):
    if isinstance(value, np.ndarray):
        return _safe(value.tolist())
    if isinstance(value, np.generic):
        return _safe(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_safe(item) for item in value]
    return value


def log_target(event, **fields):
    log_timing(event, **_safe(fields))


def target_snapshot(target):
    return {key: target.get(key) for key in (
        "target_id", "confidence", "box", "position_camera_m",
        "position_world_m", "world_frame", "_localization_context",
    )}


def position_difference(reference, current):
    delta = np.asarray(current, dtype=float) - np.asarray(reference, dtype=float)
    return {"reference_position_world_m": reference, "delta_xyz_m": delta,
            "distance_m": float(np.linalg.norm(delta)),
            "max_axis_difference_m": float(np.max(np.abs(delta)))}
