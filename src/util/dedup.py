import numpy as np

from util.target_log import log_target, target_snapshot, position_difference


DEDUP_DISTANCE_M = 1.8
MAX_CAPTURE_ATTEMPTS = 3
MAX_MATCHED_POSITIONS = 10

# Coverage/refresh matching is read-only. New IDs are admitted separately,
# only after the entire original batch is confirmed at the current hold.
RECHECK_MATCH_DISTANCE_M = 1.8


def match_rechecked_targets(targets, detections, img, log_context=None):
    """Return only mutually unique A-to-B matches, without mutating any record.

    Include the whole original batch in targets, even already visited targets,
    so their detections cannot silently be reassigned to another batch member.
    """
    candidates = {}
    owners = {}
    comparisons = {}
    height, width = img.shape[:2]
    for target_index, target in enumerate(targets):
        original = np.asarray(target["position_world_m"], dtype=float)
        if original.shape != (3,) or not np.isfinite(original).all():
            continue
        for index, detection in enumerate(detections):
            if detection.get("world_frame") != target["world_frame"]:
                comparisons.setdefault(target_index, []).append({
                    "detection_index": index, "detection": target_snapshot(detection),
                    "reason": "world_frame_mismatch", "within_gate": False})
                continue
            position = np.asarray(detection.get("position_world_m"), dtype=float)
            box = np.asarray(detection.get("box"), dtype=float)
            if position.shape != (3,) or not np.isfinite(position).all() or box.shape != (4,):
                continue
            if not (0 <= box[0] < box[2] <= width and 0 <= box[1] < box[3] <= height):
                continue
            within_gate = bool(np.linalg.norm(position - original) <= RECHECK_MATCH_DISTANCE_M)
            comparisons.setdefault(target_index, []).append({
                "detection_index": index, "detection": target_snapshot(detection),
                **position_difference(original, position), "within_gate": within_gate,
                "reason": "inside_distance_gate" if within_gate else "outside_distance_gate"})
            if not within_gate:
                continue
            candidates.setdefault(target_index, []).append(index)
            owners.setdefault(index, []).append(target_index)

    matched, missing = [], []
    for target_index, target in enumerate(targets):
        choices = candidates.get(target_index, [])
        unique = len(choices) == 1 and len(owners[choices[0]]) == 1
        log_target(
            "uav.recheck_match", context=log_context, original=target_snapshot(target),
            metric="euclidean", threshold_m=RECHECK_MATCH_DISTANCE_M,
            comparisons=comparisons.get(target_index, []), candidate_indices=choices,
            candidate_owner_ids={str(index): [targets[i]["target_id"] for i in owners[index]] for index in choices},
            decision="matched" if unique else "unconfirmed",
            reason=("mutually_unique" if unique else
                    "no_candidate_within_gate" if not choices else "ambiguous_candidates"),
        )
        if not unique:
            missing.append(target)
            continue
        detection = detections[choices[0]]
        matched.append({
            **detection, "target_id": target["target_id"],
            "_record": target["_record"], "exposure_img": img,
            "_recheck_detection_index": choices[0],
        })
    return matched, missing


def select_current_targets(
    confirmed,
    detections,
    img,
    target_records,
    log_context=None,
):
    """Commit a fully covered B batch, preserving A IDs and admitting B extras.

    Call only after full coverage and a stable-pose check. Never allocate B-only
    records on the return-to-A branch. Ambiguous extras stay retryable on patrol.
    """
    used = {target["_recheck_detection_index"] for target in confirmed}
    protected_ids = {target["target_id"] for target in confirmed}
    height, width = img.shape[:2]
    extras = []
    for index, detection in enumerate(detections):
        if index in used:
            continue
        box = np.asarray(detection.get("box"), dtype=float)
        position = np.asarray(detection.get("position_world_m"), dtype=float)
        if (
            box.shape != (4,)
            or not np.isfinite(box).all()
            or not (0 <= box[0] < box[2] <= width and 0 <= box[1] < box[3] <= height)
            or position.shape != (3,)
            or not np.isfinite(position).all()
        ):
            continue
        histories = target_records.get(detection["world_frame"], [])
        candidates = [record for record in histories if any(
            np.all(np.abs(position - old) < DEDUP_DISTANCE_M)
            for old in record["positions"]
        )]
        overlaps_confirmed = any(
            detection["world_frame"] == target["world_frame"]
            and (
                np.linalg.norm(position - np.asarray(target["position_world_m"])) <= RECHECK_MATCH_DISTANCE_M
                or np.all(np.abs(position - np.asarray(target["position_world_m"])) < DEDUP_DISTANCE_M)
            )
            for target in confirmed
        )
        if (
            overlaps_confirmed
            or len(candidates) > 1
            or any(record["target_id"] in protected_ids for record in candidates)
        ):
            log_target(
                "uav.current_extra_skipped",
                context=log_context,
                detection_index=index,
                detection=target_snapshot(detection),
                reason="overlaps_confirmed_or_ambiguous_history",
                candidate_target_ids=[record["target_id"] for record in candidates],
            )
            continue
        extras.append(dict(detection))

    selected = select_new_targets(
        extras,
        target_records,
        log_context=log_context,
    )
    for target in confirmed:
        positions = target["_record"]["positions"]
        positions.append(np.asarray(target["position_world_m"], dtype=float).copy())
        del positions[:-MAX_MATCHED_POSITIONS]
    batch = list(confirmed) + selected
    for target in batch:
        target["exposure_img"] = img
        target["_localization_context"] = log_context
    log_target(
        "uav.current_capture_batch",
        context=log_context,
        original_target_ids=[target["target_id"] for target in confirmed],
        extra_target_ids=[target["target_id"] for target in selected],
        targets=[target_snapshot(target) for target in batch],
    )
    return batch


def select_new_targets(detections, target_records, log_context=None):
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
        comparisons = []
        for item in records:
            if not item["positions"]:
                continue
            reference = min(item["positions"], key=lambda old: np.max(np.abs(position - old)))
            comparisons.append({
                "target_id": item["target_id"], "observed": item["observed"],
                "attempts": item["attempts"], "history_count": len(item["positions"]),
                **position_difference(reference, position),
                "within_gate": bool(np.all(np.abs(position - reference) < DEDUP_DISTANCE_M)),
            })
        is_new = record is None
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
        reason = ("already_observed" if record["observed"] else
                  "attempt_limit" if record["attempts"] >= MAX_CAPTURE_ATTEMPTS else
                  "already_selected_in_batch" if record["target_id"] in selected_ids else
                  "new_target" if is_new else "retry_existing_target")
        log_target(
            "uav.dedup_match", context=log_context, detection=target_snapshot(detection),
            target_id=record["target_id"], new_record=is_new,
            metric="each_axis_strictly_less", threshold_m=DEDUP_DISTANCE_M,
            comparisons=comparisons, reason=reason,
            decision="skip" if reason in ("already_observed", "attempt_limit", "already_selected_in_batch") else "select",
        )
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
