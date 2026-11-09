"""Choose B only on full A coverage, then photograph the chosen pending batch."""

from functools import partial

import rospy

from task.autofocus import prepare_camera_for_detection, run as run_autofocus
from util.timing import timed, log_timing
from util.target_log import log_target, target_snapshot


def capture_target(detect_thread, target, capture, frame_source):
    target_id = target["target_id"]
    rospy.loginfo(f"Target {target_id} autofocus: frame_source={frame_source}, box={target['box']}")
    log_target("uav.target_capture_start", frame_source=frame_source, target=target_snapshot(target))
    captured = run_autofocus(
        target["exposure_img"], target["box"],
        capture=partial(capture, target_id), target_id=target_id,
        camera_prepared=frame_source == "current_recheck",
    )
    if captured:
        detect_thread.mark_observed(target)
    log_target("uav.target_capture_result", target_id=target_id, captured=bool(captured),
               observed=target["_record"]["observed"], frame_source=frame_source)
    return captured


@timed("uav.observe_current_or_return")
def run(flight, detect_thread, exposure_pose, targets, capture):
    # find_targets() has already stopped its producer. Sending the CURRENT
    # pose acquires mission control; pausing image sampling alone cannot hover.
    hold_pose = flight.hold_current_pose()
    log_target("uav.observation_positions", original_sampling_pose=exposure_pose,
               current_hold_pose=hold_pose, targets=[target_snapshot(target) for target in targets])
    if hold_pose["frame_id"] != exposure_pose["frame_id"]:
        raise RuntimeError("SLAM frame changed since original detection; refusing to revisit old coordinates")
    prepare_camera_for_detection()
    matched, missing = detect_thread.recheck_targets(targets)
    flight.assert_at_pose(hold_pose)
    log_timing(
        "uav.recheck_decision", phase="current_pose",
        matched_target_ids=[target["target_id"] for target in matched],
        unconfirmed_target_ids=[target["target_id"] for target in missing],
        fallback_target_ids=[target["target_id"] for target in targets] if missing else [],
    )

    if not missing:
        # Only now commit B-only IDs/history. A partial B never adds pending IDs.
        batch = detect_thread.expand_current_targets(matched)
        log_target(
            "uav.capture_location_selected",
            location="B",
            reason="all_original_targets_covered",
            pose=hold_pose,
            target_ids=[target["target_id"] for target in batch],
        )
        for index, target in enumerate(batch):
            flight.wait_for_stable_pose(hold_pose)
            flight.assert_at_pose(hold_pose)
            confirmed = target
            # Each autofocus moves/restores the camera. Refresh later targets
            # against the entire fixed B batch, including completed members.
            if index:
                refreshed, _ = detect_thread.recheck_targets(batch)
                flight.assert_at_pose(hold_pose)
                confirmed = next((item for item in refreshed if item["target_id"] == target["target_id"]), None)
                if confirmed is None:
                    log_target(
                        "uav.recheck_later_missing",
                        target_id=target["target_id"],
                        reason="not_confirmed_on_refresh_retry_on_patrol",
                        original=target_snapshot(target),
                    )
                    continue
            detect_thread.start_attempt(confirmed)
            capture_target(
                detect_thread,
                confirmed,
                capture,
                frame_source="current_recheck",
            )
        detect_thread.last_recheck = None
        return

    # No local capture on partial coverage: return the entire A batch once.
    detect_thread.pause()
    detect_thread.last_recheck = None
    log_target(
        "uav.capture_location_selected",
        location="A",
        reason="original_batch_not_fully_covered",
        pose=exposure_pose,
        target_ids=[target["target_id"] for target in targets],
        unconfirmed_target_ids=[target["target_id"] for target in missing],
    )
    rospy.loginfo("Returning to original sampling pose for the entire target batch")
    log_target("uav.return_for_unconfirmed_targets", current_hold_pose=hold_pose,
               destination_pose=exposure_pose, target_ids=[target["target_id"] for target in targets],
               originals=[target_snapshot(target) for target in targets],
               reason="original_batch_not_fully_covered_see_recheck_match_or_unavailable")
    flight.go_to_pose(exposure_pose)
    for original in targets:
        target_id = original["target_id"]
        flight.wait_for_stable_pose(exposure_pose)
        flight.assert_at_pose(exposure_pose)
        # At the original sampling pose use the original detection image/box,
        # as in the previous workflow. SAM2 then tracks newly acquired frames;
        # there is no additional YOLO/depth request to confirm the return view.
        detect_thread.start_attempt(original)
        log_timing(
            "uav.recheck_decision", phase="original_pose", target_id=target_id,
            frame_source="original_detection",
        )
        capture_target(detect_thread, original, capture, frame_source="original_detection")
