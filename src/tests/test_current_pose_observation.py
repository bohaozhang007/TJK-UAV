"""Offline regression tests; hardware/ROS boundaries are replaced before imports.

python -m unittest discover -s src/tests -v
"""

import sys
import time
import types
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def stub(name, **fields):
    module = types.ModuleType(name)
    module.__dict__.update(fields)
    sys.modules[name] = module
    return module


class Interrupted(Exception):
    pass


ros = stub(
    "rospy", ROSInterruptException=Interrupted, is_shutdown=Mock(return_value=False),
    sleep=Mock(), loginfo=Mock(), logwarn=Mock(), logerr=Mock(),
    logwarn_throttle=Mock(), init_node=Mock(), on_shutdown=Mock(),
    signal_shutdown=Mock(), Subscriber=Mock(),
    Time=types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_sec=lambda: time.time())),
)
stub("hardware.k40t", **{name: Mock() for name in (
    "get_img", "detect_img", "track_img", "get_gimbal", "set_gimbal",
    "get_zoom", "set_zoom", "check_tracker", "close_camera", "disable_osd",
)})
stub("hardware.pose", get_pose=Mock(), close_pose=Mock())
for module, name in (
    ("check_goal_reached", "GoalReached"), ("release_control", "ControlRelease"),
    ("to_target_pose", "TargetPose"), ("next_waypoint", "NextWaypoint"),
):
    stub("interface." + module, **{name: Mock()})
stub("mavros_msgs")
stub("mavros_msgs.msg", State=object)

from util.dedup import match_rechecked_targets, select_new_targets, select_current_targets
from util import flight as flight_module
from task import detect_thread as detection_module
from task import observe_targets as observation
from task import autofocus as autofocus_module
from task import main as mission


def pose(x=0, yaw=0, frame="world"):
    return {"frame_id": frame, "position_m": [x, 0, 1], "yaw_deg": yaw,
            "stamp_s": time.time(), "quaternion_xyzw": [0, 0, 0, 1]}


def target(identity=1, x=5):
    return {"target_id": identity, "world_frame": "world", "position_world_m": [x, 0, 1],
            "box": [10, 10, 20, 30], "exposure_img": np.zeros((64, 64, 3), dtype=np.uint8),
            "_record": {"target_id": identity, "positions": [np.array([x, 0, 1.])],
                        "attempts": 0, "observed": False}}


def confirmed(original):
    return {**original, "box": [22, 12, 32, 34],
            "exposure_img": np.ones((64, 64, 3), dtype=np.uint8)}


def detection_for(original):
    return {key: confirmed(original)[key] for key in ("position_world_m", "world_frame", "box")}


class MatchingTests(unittest.TestCase):
    def test_matches_keep_original_id_record_and_new_image_without_mutation(self):
        original = target()
        current = confirmed(original)
        detection = {key: current[key] for key in ("box", "world_frame", "position_world_m")}
        matched, missing = match_rechecked_targets([original], [detection], current["exposure_img"])
        self.assertFalse(missing)
        self.assertEqual(matched[0]["target_id"], 1)
        self.assertIs(matched[0]["_record"], original["_record"])
        self.assertIs(matched[0]["exposure_img"], current["exposure_img"])
        self.assertNotIn("target_id", detection)
        self.assertEqual(original["_record"]["attempts"], 0)
        self.assertEqual(len(original["_record"]["positions"]), 1)

    def test_rejects_ambiguous_targets_and_ambiguous_detections(self):
        original = target()
        other = target(2, x=6)
        box = confirmed(original)
        for references, detections in (([original, other], [box]), ([original], [box, dict(box)])):
            matched, missing = match_rechecked_targets(references, detections, box["exposure_img"])
            self.assertFalse(matched)
            self.assertEqual(len(missing), len(references))

    def test_rejects_wrong_frame_invalid_depth_box_and_distant_target(self):
        original = target()
        for updates in (
            {"world_frame": "other"}, {"position_world_m": None},
            {"position_world_m": [float("nan"), 0, 1]},
            {"position_world_m": [6.5, 1.5, 1]}, {"box": [0, 0, 1000, 20]},
        ):
            with self.subTest(updates=updates):
                detection = {**confirmed(original), **updates}
                matched, missing = match_rechecked_targets([original], [detection], detection["exposure_img"])
                self.assertFalse(matched)
                self.assertEqual(missing, [original])


class DetectionTests(unittest.TestCase):
    def setUp(self):
        ros.is_shutdown.return_value = False
        self.detector = detection_module.DetectThread.__new__(detection_module.DetectThread)
        self.detector.worker = None
        self.detector.recheck_frame_id = 0
        self.detector.target_records = {}

    def test_recheck_samples_new_image_and_pose_without_allocating_records(self):
        original = target()
        image = np.ones((64, 64, 3), dtype=np.uint8)
        current_pose = pose(2)
        with patch.object(detection_module, "capture_current_frame", return_value=(image, current_pose)), \
             patch.object(detection_module, "detect_img", return_value=[detection_for(original)]) as detect:
            matched, missing = self.detector.recheck_targets([original])
        self.assertFalse(missing)
        self.assertIs(matched[0]["exposure_img"], image)
        self.assertIs(detect.call_args.args[0], image)
        self.assertIs(detect.call_args.args[1], current_pose)
        self.assertTrue(detect.call_args.kwargs["calculate_depth"])
        self.assertEqual(detect.call_args.kwargs["frame_id"], -1)
        self.assertEqual(self.detector.target_records, {})

    def test_retry_recaptures_image_and_pose(self):
        original = target()
        first, second = original["exposure_img"], confirmed(original)["exposure_img"]
        with patch.object(detection_module, "capture_current_frame", side_effect=[(first, pose()), (second, pose(2))]) as capture, \
             patch.object(detection_module, "detect_img", side_effect=[RuntimeError("network error"), [detection_for(original)]]) as detect:
            matched, _ = self.detector.recheck_targets([original])
        self.assertEqual(capture.call_count, 2)
        self.assertIs(matched[0]["exposure_img"], second)
        self.assertEqual(detect.call_args.kwargs["frame_id"], -2)

    def test_total_failure_returns_unconfirmed_and_active_patrol_is_rejected(self):
        original = target()
        with patch.object(detection_module, "capture_current_frame", side_effect=RuntimeError("camera unavailable")):
            matched, missing = self.detector.recheck_targets([original])
        self.assertFalse(matched)
        self.assertEqual(missing, [original])
        self.detector.worker = object()
        with self.assertRaises(RuntimeError):
            self.detector.recheck_targets([original])

    def test_sampling_rejects_stale_pairs_and_never_reads_patrol_queue(self):
        with patch.object(detection_module, "get_img", return_value=(object(), 0)) as image, \
             patch.object(detection_module, "get_pose", return_value=(pose(), 0)), \
             patch.object(detection_module, "sample_is_fresh", return_value=False):
            with self.assertRaises(RuntimeError):
                detection_module.capture_current_frame()
            self.assertEqual(image.call_count, detection_module.RECHECK_CAPTURE_ATTEMPTS)

    def test_fresh_capture_returns_the_exact_paired_samples(self):
        image, current_pose = object(), pose(3)
        with patch.object(detection_module, "get_img", return_value=(image, 1)), \
             patch.object(detection_module, "get_pose", return_value=(current_pose, 1)), \
             patch.object(detection_module, "sample_is_fresh", return_value=True):
            result_image, result_pose = detection_module.capture_current_frame()
        self.assertIs(result_image, image)
        self.assertIs(result_pose, current_pose)

    def test_range_filter_uses_current_paired_pose_and_rejects_invalid_localization(self):
        far = confirmed(target(x=20))
        near = confirmed(target(x=5))
        invalid = {**near, "position_world_m": None}
        kept = detection_module.filter_localized_detections([far, near, invalid], pose(2))
        self.assertEqual(len(kept), 1)
        self.assertIs(kept[0], near)


class FlightTests(unittest.TestCase):
    def setUp(self):
        ros.is_shutdown.return_value = False
        self.flight = flight_module.FlightControl.__new__(flight_module.FlightControl)

    def test_hold_acquires_control_using_current_pose_before_waiting(self):
        current = pose(3)
        self.flight.go_to_pose = Mock()
        self.flight.wait_for_stable_pose = Mock()
        with patch.object(flight_module, "get_pose", return_value=(current, 0)):
            result = self.flight.hold_current_pose()
        self.assertIs(result, current)
        self.flight.go_to_pose.assert_called_once_with(current)
        self.flight.wait_for_stable_pose.assert_called_once_with(current)

    def test_requires_settling_not_just_a_single_arrival_ack(self):
        samples = [(pose(x), 0) for x in (1, 0.2, 0.21, 0.22, 0.21)]
        with patch.object(flight_module, "get_pose", side_effect=samples) as get:
            self.flight.wait_for_stable_pose(pose())
        self.assertEqual(get.call_count, 5)

    def test_hold_timeout_and_drift_prevent_tracking(self):
        with patch.object(flight_module.time, "monotonic", side_effect=[0, 11]):
            with self.assertRaises(RuntimeError):
                self.flight.wait_for_stable_pose(pose())
        with patch.object(flight_module, "get_pose", return_value=(pose(1), 0)):
            with self.assertRaises(RuntimeError):
                self.flight.assert_at_pose(pose())

    def test_frame_and_wrapped_yaw_checks(self):
        self.assertTrue(flight_module.poses_close(pose(yaw=179), pose(yaw=-179)))
        self.assertFalse(flight_module.poses_close(pose(frame="other"), pose()))
        self.assertFalse(flight_module.poses_close(pose(yaw=20), pose()))


class WorkflowTests(unittest.TestCase):
    def setUp(self):
        ros.is_shutdown.return_value = False
        self.flight = Mock(spec=flight_module.FlightControl)
        self.flight.hold_current_pose.return_value = pose(3)
        self.detector = Mock()
        self.detector.expand_current_targets.side_effect = lambda matched: list(matched)
        self.detector.start_attempt.side_effect = lambda t: t["_record"].__setitem__("attempts", t["_record"]["attempts"] + 1)
        self.detector.mark_observed.side_effect = lambda t: t["_record"].__setitem__("observed", True)
        self.capture = Mock()
        self.original = target()
        self.return_pose = pose()
        self.prepare = patch.object(observation, "prepare_camera_for_detection").start()
        self.autofocus = patch.object(observation, "run_autofocus", return_value=True).start()
        self.addCleanup(patch.stopall)

    def run_batch(self, targets=None):
        observation.run(self.flight, self.detector, self.return_pose, targets or [self.original], self.capture)

    def test_current_match_uses_new_frame_and_original_id_without_return(self):
        fresh = confirmed(self.original)
        order = []
        self.flight.hold_current_pose.side_effect = lambda: order.append("hold") or pose(3)
        self.detector.recheck_targets.side_effect = lambda refs: (order.append("recheck") or [fresh], [])
        self.run_batch()
        self.assertEqual(order, ["hold", "recheck"])
        self.flight.go_to_pose.assert_not_called()
        self.assertIs(self.autofocus.call_args.args[0], fresh["exposure_img"])
        self.assertEqual(self.autofocus.call_args.args[1], fresh["box"])
        self.assertTrue(self.autofocus.call_args.kwargs["camera_prepared"])
        self.autofocus.call_args.kwargs["capture"]()
        self.capture.assert_called_once_with(1)
        self.assertTrue(self.original["_record"]["observed"])

    def test_absent_target_returns_and_uses_original_image_without_redetection(self):
        self.detector.recheck_targets.return_value = ([], [self.original])
        self.run_batch()
        self.flight.go_to_pose.assert_called_once_with(self.return_pose)
        self.assertEqual(self.detector.recheck_targets.call_count, 1)
        self.assertIs(self.autofocus.call_args.args[0], self.original["exposure_img"])
        self.assertEqual(self.autofocus.call_args.args[1], self.original["box"])
        self.assertFalse(self.autofocus.call_args.kwargs["camera_prepared"])
        self.assertTrue(self.original["_record"]["observed"])

    def test_failed_return_tracking_is_not_marked_observed(self):
        self.detector.recheck_targets.return_value = ([], [self.original])
        self.autofocus.return_value = False
        self.run_batch()
        self.autofocus.assert_called_once()
        self.assertEqual(self.detector.recheck_targets.call_count, 1)
        self.assertFalse(self.original["_record"]["observed"])
        self.assertEqual(self.original["_record"]["attempts"], 1)

    def test_two_local_targets_refresh_second_frame(self):
        other = target(2, x=10)
        first, second, later = confirmed(self.original), confirmed(other), confirmed(other)
        self.detector.recheck_targets.side_effect = [([first, second], []), ([first, later], [])]
        self.run_batch([self.original, other])
        self.flight.go_to_pose.assert_not_called()
        self.assertEqual(self.autofocus.call_count, 2)
        self.assertIs(self.autofocus.call_args_list[1].args[0], later["exposure_img"])
        self.assertTrue(other["_record"]["observed"])
        self.assertEqual(self.detector.recheck_targets.call_args.args[0], [first, second])

    def test_mixed_batch_returns_before_capturing_entire_original_batch(self):
        other = target(2, x=10)
        first = confirmed(self.original)
        self.detector.recheck_targets.return_value = ([first], [other])
        order = []
        self.autofocus.side_effect = lambda *args, **kwargs: order.append(kwargs["target_id"]) or True
        self.flight.go_to_pose.side_effect = lambda p: order.append("return")
        self.run_batch([self.original, other])
        self.assertEqual(order, ["return", 1, 2])
        self.detector.expand_current_targets.assert_not_called()
        self.detector.pause.assert_called_once()
        self.assertIs(self.autofocus.call_args_list[0].args[0], self.original["exposure_img"])
        self.assertEqual(self.detector.recheck_targets.call_count, 1)
        self.assertIs(self.autofocus.call_args_list[-1].args[0], other["exposure_img"])

    def test_target_disappearing_at_chosen_b_stays_pending_without_return(self):
        other = target(2, x=10)
        first, second = confirmed(self.original), confirmed(other)
        self.detector.recheck_targets.side_effect = [([first, second], []), ([first], [other])]
        self.run_batch([self.original, other])
        self.flight.go_to_pose.assert_not_called()
        self.assertEqual(self.detector.recheck_targets.call_count, 2)
        self.assertEqual(self.autofocus.call_count, 1)
        self.assertFalse(other["_record"]["observed"])
        self.assertEqual(other["_record"]["attempts"], 0)

    def test_tracking_failure_leaves_original_record_retryable(self):
        self.detector.recheck_targets.return_value = ([confirmed(self.original)], [])
        self.autofocus.return_value = False
        self.run_batch()
        self.assertFalse(self.original["_record"]["observed"])
        self.assertEqual(self.original["_record"]["attempts"], 1)

    def test_drift_during_windows_request_aborts_before_autofocus(self):
        self.detector.recheck_targets.return_value = ([confirmed(self.original)], [])
        self.flight.assert_at_pose.side_effect = RuntimeError("drift")
        with self.assertRaises(RuntimeError):
            self.run_batch()
        self.autofocus.assert_not_called()
        self.assertFalse(self.original["_record"]["observed"])

    def test_changed_slam_frame_prevents_recheck_and_return_to_old_frame(self):
        self.flight.hold_current_pose.return_value = pose(3, frame="new_world")
        with self.assertRaises(RuntimeError):
            self.run_batch()
        self.detector.recheck_targets.assert_not_called()
        self.flight.go_to_pose.assert_not_called()


class FullCoverageBatchTests(unittest.TestCase):
    def setUp(self):
        ros.is_shutdown.return_value = False
        self.detector = detection_module.DetectThread()
        self.img = np.zeros((64, 64, 3), dtype=np.uint8)
        self.a = select_new_targets(
            [self.raw(2), self.raw(5)], self.detector.target_records,
        )
        for item in self.a:
            item["exposure_img"] = self.img
        self.flight = Mock(spec=flight_module.FlightControl)
        self.flight.hold_current_pose.return_value = pose(3)
        self.capture = Mock()

    def raw(self, x):
        return {"position_world_m": [x, 0, 1], "world_frame": "world",
                "box": [10, 10, 20, 30], "confidence": 0.8}

    def run_batch(self, detections, autofocus_result=True):
        with patch.object(detection_module, "capture_current_frame", side_effect=lambda: (self.img.copy(), pose(3))), \
             patch.object(detection_module, "detect_img", side_effect=lambda *args, **kwargs: [dict(item) for item in detections]), \
             patch.object(observation, "prepare_camera_for_detection"), \
             patch.object(observation, "run_autofocus", return_value=autofocus_result) as autofocus:
            observation.run(self.flight, self.detector, pose(), self.a, self.capture)
        return autofocus

    def test_a12_b123_captures_all_three_at_b_with_preserved_ids(self):
        autofocus = self.run_batch([self.raw(2.1), self.raw(5.1), self.raw(8)])
        self.flight.go_to_pose.assert_not_called()
        self.assertEqual([call.kwargs["target_id"] for call in autofocus.call_args_list], [1, 2, 3])
        self.assertTrue(all(call.kwargs["camera_prepared"] for call in autofocus.call_args_list))
        records = self.detector.target_records["world"]
        self.assertEqual(len(records), 3)
        self.assertTrue(all(record["observed"] for record in records))
        self.assertTrue(all(record["attempts"] == 1 for record in records))
        self.assertIsNot(autofocus.call_args_list[0].args[0], autofocus.call_args_list[1].args[0])
        self.assertEqual(records[0]["positions"][-1][0], 2.1)

    def test_partial_b_returns_all_a_without_allocating_b_only_id(self):
        order = []
        self.flight.go_to_pose.side_effect = lambda p: order.append("return")
        with patch.object(self.detector, "start_attempt", wraps=self.detector.start_attempt) as start:
            start.side_effect = lambda t: order.append(t["target_id"])
            autofocus = self.run_batch([self.raw(2.1), self.raw(8)])
        self.assertEqual(order, ["return", 1, 2])
        self.assertEqual(len(self.detector.target_records["world"]), 2)
        self.assertTrue(all(call.args[0] is self.img for call in autofocus.call_args_list))
        self.assertTrue(all(not call.kwargs["camera_prepared"] for call in autofocus.call_args_list))

    def test_b_historical_completed_and_exhausted_targets_are_not_admitted(self):
        old = select_new_targets([self.raw(8), self.raw(-2)], self.detector.target_records)
        old[0]["_record"]["observed"] = True
        old[1]["_record"]["attempts"] = 3
        autofocus = self.run_batch([self.raw(2.1), self.raw(5.1), self.raw(8), self.raw(-2)])
        self.assertEqual([call.kwargs["target_id"] for call in autofocus.call_args_list], [1, 2])
        self.assertEqual(len(self.detector.target_records["world"]), 4)

    def test_failed_b_captures_leave_all_original_and_extra_records_retryable(self):
        self.run_batch([self.raw(2.1), self.raw(5.1), self.raw(8)], autofocus_result=False)
        records = self.detector.target_records["world"]
        self.assertEqual(len(records), 3)
        self.assertTrue(all(not record["observed"] and record["attempts"] == 1 for record in records))

    def test_b_extra_unfinished_history_keeps_existing_id(self):
        pending = select_new_targets([self.raw(8)], self.detector.target_records)[0]
        pending["_record"]["attempts"] = 1
        autofocus = self.run_batch([self.raw(2.1), self.raw(5.1), self.raw(8.2)])
        self.assertEqual([call.kwargs["target_id"] for call in autofocus.call_args_list], [1, 2, 3])
        self.assertEqual(len(self.detector.target_records["world"]), 3)
        self.assertEqual(pending["_record"]["attempts"], 2)
        self.assertTrue(pending["_record"]["observed"])

    def test_ambiguous_coverage_returns_all_a_and_leaves_histories_unchanged(self):
        self.run_batch([self.raw(2), self.raw(2.1), self.raw(5.1), self.raw(8)])
        self.flight.go_to_pose.assert_called_once()
        self.assertEqual(len(self.detector.target_records["world"]), 2)
        self.assertTrue(all(len(record["positions"]) == 1 for record in self.detector.target_records["world"]))

    def test_drift_before_admission_does_not_create_extra_records(self):
        self.flight.assert_at_pose.side_effect = RuntimeError("drift")
        with self.assertRaises(RuntimeError):
            self.run_batch([self.raw(2.1), self.raw(5.1), self.raw(8)])
        self.assertEqual(len(self.detector.target_records["world"]), 2)

    def test_expansion_rejects_missing_coverage_or_failed_request(self):
        self.detector.last_recheck = None
        with self.assertRaises(RuntimeError):
            self.detector.expand_current_targets([])
        self.detector.last_recheck = {"missing_ids": [2], "matched_ids": {1}}
        with self.assertRaises(RuntimeError):
            self.detector.expand_current_targets([self.a[0]])

    def test_b_extra_matching_multiple_history_records_is_deferred(self):
        # History overlap may occur after localization drift; never choose an
        # arbitrary completed/unfinished identity for a B-only detection.
        history = self.detector.target_records["world"]
        history.extend([
            {"target_id": 3, "positions": [np.array([9., 0, 1])], "attempts": 0, "observed": False},
            {"target_id": 4, "positions": [np.array([11., 0, 1])], "attempts": 0, "observed": True},
        ])
        raw = [self.raw(2.1), self.raw(5.1), self.raw(10)]
        matched, missing = match_rechecked_targets(self.a, raw, self.img)
        self.assertFalse(missing)
        chosen = select_current_targets(matched, raw, self.img, self.detector.target_records)
        self.assertEqual([item["target_id"] for item in chosen], [1, 2])
        self.assertEqual(len(history), 4)


class EntryPointTests(unittest.TestCase):
    def test_non_autofocus_observation_point_modes_are_preserved(self):
        for skip in (False, True):
            with self.subTest(skip_exposure_point=skip):
                stopped = [False]
                flight, detector = Mock(), Mock()
                original = target()
                initial_pose, visit_pose = pose(), pose(6)
                detector.find_targets.return_value = (initial_pose, {}, [original])
                flight.release_control.side_effect = lambda: stopped.__setitem__(0, True)

                def subscribe(topic, kind, callback, **kwargs):
                    callback(types.SimpleNamespace(mode="OFFBOARD"))
                    return Mock()

                with patch.object(mission, "FlightControl", return_value=flight), \
                     patch.object(mission, "DetectThread", return_value=detector), \
                     patch.object(mission, "observe_current_or_return") as observe, \
                     patch.object(mission, "prepare_observation_points", return_value=[{"target": original, "pose": visit_pose}]), \
                     patch.object(mission, "capture_images") as capture, \
                     patch.object(ros, "Subscriber", side_effect=subscribe), \
                     patch.object(ros, "is_shutdown", side_effect=lambda: stopped[0]):
                    mission.main(autofocus=False, skip_exposure_point=skip)
                observe.assert_not_called()
                capture.assert_called_once_with(1)
                poses = [call.args[0] for call in flight.go_to_pose.call_args_list]
                self.assertEqual(poses, [visit_pose, initial_pose] if skip else [initial_pose, visit_pose, initial_pose])

    def test_autofocus_routes_to_current_pose_workflow_and_releases_afterward(self):
        stopped = [False]
        flight, detector = Mock(), Mock()
        detector.find_targets.return_value = (pose(), {}, [target()])
        flight.release_control.side_effect = lambda: stopped.__setitem__(0, True)

        def subscribe(topic, kind, callback, **kwargs):
            callback(types.SimpleNamespace(mode="OFFBOARD"))
            return Mock()

        with patch.object(mission, "FlightControl", return_value=flight), \
             patch.object(mission, "DetectThread", return_value=detector), \
             patch.object(mission, "observe_current_or_return") as observe, \
             patch.object(ros, "Subscriber", side_effect=subscribe), \
             patch.object(ros, "is_shutdown", side_effect=lambda: stopped[0]):
            mission.main(autofocus=True)
        observe.assert_called_once()
        flight.go_to_pose.assert_not_called()
        flight.release_control.assert_called_once()

    def test_prepared_camera_does_not_move_before_tracker_initialization(self):
        original = target()
        with patch.object(autofocus_module, "prepare_camera_for_detection") as prepare, \
             patch.object(autofocus_module, "track_img", return_value={"box": None}) as track:
            self.assertFalse(autofocus_module.run(original["exposure_img"], original["box"], camera_prepared=True))
        prepare.assert_not_called()
        track.assert_called_once()


if __name__ == "__main__":
    unittest.main()
