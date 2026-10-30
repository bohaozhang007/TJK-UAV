import numpy as np


DEDUP_DISTANCE_M = 1.8
MAX_CAPTURE_ATTEMPTS = 3
MAX_MATCHED_POSITIONS = 10


def select_new_targets(detections, target_records):
    """Match target histories and select at most one attempt per target per batch."""
    selected = []
    selected_ids = set()
    for detection in detections:
        if detection["position_world_m"] is None:
            continue
        position = np.asarray(detection["position_world_m"], dtype=float)
        if (
            position.shape != (3,)
            or not np.isfinite(position).all()
        ):
            continue
        records = target_records.setdefault(detection["world_frame"], [])
        record = next((
            item for item in records
            if any(np.all(np.abs(position - old) < DEDUP_DISTANCE_M) for old in item["positions"])
        ), None)
        if record is None:
            record = {
                "target_id": sum(len(items) for items in target_records.values()) + 1,
                "positions": [],
                "attempts": 0,
                "observed": False,
            }
            records.append(record)
        record["positions"].append(position.copy())
        del record["positions"][:-MAX_MATCHED_POSITIONS]
        if (
            record["observed"]
            or record["attempts"] >= MAX_CAPTURE_ATTEMPTS
            or record["target_id"] in selected_ids
        ):
            continue
        detection["target_id"] = record["target_id"]
        detection["_record"] = record
        selected_ids.add(record["target_id"])
        selected.append(detection)
    return selected
