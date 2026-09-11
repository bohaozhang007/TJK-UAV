"""Route patrol with asynchronous perception; reuse v20 visual TRACK control."""
from __future__ import annotations

import argparse
import datetime as dt
import json
import math
from pathlib import Path
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import cv2
import numpy as np

from agent.tjk import v20
from agent.tjk.patrol import PerceptionPipeline, TargetGeometryError, TargetMemory, target_position
from robot_client.owl_ego import OwlEgoClient


def validate_patrol_config(config):
    """Validate new settings before creating clients/models or taking control."""
    section = v20._config_section(config, "patrol")
    spec = {
        "capture_fps": (0.1, 60, False), "det_interval": (1, 10000, True),
        "poll_interval_s": (0.02, 1, False), "navigation_timeout_s": (1, 3600, False),
        "cancel_timeout_s": (1, 120, False), "worker_idle_timeout_s": (1, 600, False),
        "max_detection_age_s": (0.1, 600, False),
        "observation_max_age_s": (0.05, 5, False),
        "observation_max_sync_error_s": (0, 0.5, False),
        "dedup_distance_cm": (1, 10000, False), "retry_cooldown_s": (0, 3600, False),
        "max_target_attempts": (1, 20, True), "reacquire_attempts": (1, 20, True),
        "min_depth_pixels": (1, 1000000, True), "max_relative_depth_mad": (0, 1, False),
        "box_core_ratio": (0.01, 1, False),
    }
    if set(section) != set(spec):
        raise ValueError(f"Invalid patrol keys: missing={set(spec)-set(section)}, "
                         f"unknown={set(section)-set(spec)}")
    parsed = {}
    for key, (minimum, maximum, integer) in spec.items():
        parsed[key] = v20._config_number(section, key, minimum=minimum,
                                       maximum=maximum, integer=integer)
        if not math.isfinite(parsed[key]):
            raise ValueError(f"patrol.{key} must be finite")
    if config["track"]["skip"] or not config["scan"]["skip"]:
        raise ValueError("v21 requires track.skip=false and scan.skip=true")
    for waypoint in config["mission"]["waypoints"]:
        if "only_arrive" in waypoint:
            raise ValueError("v21 waypoints define a route; remove v20 only_arrive")
    return parsed


class PatrolAgent(v20.TJKAgent):
    def __init__(self, *, config, **kwargs):
        self.patrol = validate_patrol_config(config)
        super().__init__(config=config, **kwargs)
        self.memory = TargetMemory(self.patrol["dedup_distance_cm"],
                                   self.patrol["retry_cooldown_s"],
                                   self.patrol["max_target_attempts"])
        self.pipeline = None
        self.active_navigation = None
        self.capture_observation = None
        self.phase = "INITIALIZING"
        self._event_lock = threading.Lock()
        self.event_path = self._mission_vis_dir.parent / "events.jsonl"
        self.client.event = self.event

    def connect(self):
        # A fresh session establishes fresh world coordinates and target memory.
        result = super().connect()
        self.memory = TargetMemory(self.patrol["dedup_distance_cm"],
                                   self.patrol["retry_cooldown_s"],
                                   self.patrol["max_target_attempts"])
        self.capture_observation = None
        return result

    def event(self, kind, **fields):
        record = {"time": dt.datetime.now().isoformat(), "event": kind, **fields}
        with self._event_lock:
            with self.event_path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
        if kind not in {"http_request", "flight_health", "navigation_status", "observation", "pose_sample"}:
            self._log(f"[V21] {kind} {fields}")

    def set_phase(self, phase, **fields):
        self.phase = phase
        self.event("phase", phase=phase, **fields)

    def _ensure_flight_safety(self, context):
        # v20's unconditional rgb_ok gate predates retryable synchronized frames.
        try:
            return self.client.flight_health()
        except Exception as exc:
            raise v20.FlightSafetyError(f"{context}: {exc}") from exc

    def _calculate_motion_error(self, start_pose, end_pose, dx, dy, dz, dyaw):
        error = super()._calculate_motion_error(start_pose, end_pose, dx, dy, dz, dyaw)
        # z=0 preserves Robot's reference; measured start Z is not that reference.
        if dz == 0:
            error["ez"] = error["epos"] = None
        self.event("relative_pose_samples", start_pose=start_pose, end_pose=end_pose,
                   command=dict(x=dx, y=dy, z=dz, yaw=dyaw),
                   zero_z_preserves_reference=dz == 0,
                   note="Separate pose samples, not Robot acceptance/terminal snapshots")
        return error

    @staticmethod
    def _format_motion_error(error):
        if error is not None and error.get("ez") is None:
            return (f"(ex={error['ex']:.2f}cm, ey={error['ey']:.2f}cm, "
                    f"ez=N/A(reference retained), eyaw={error['eyaw']:.2f}deg, epos=N/A)")
        return v20.TJKAgent._format_motion_error(error)

    def _capture_for_task(self, include_depth, context):
        result = super()._capture_for_task(include_depth, context)
        self.capture_observation = self.client.last_observation
        return result

    def position(self, obs, depth, box, mask=None):
        return target_position(obs, depth, box, mask=mask,
                               min_pixels=self.patrol["min_depth_pixels"],
                               max_relative_mad=self.patrol["max_relative_depth_mad"],
                               core_ratio=self.patrol["box_core_ratio"])

    def infer_observation(self, obs):
        start = time.monotonic()
        detections = self.detect(obs.rgb)
        candidates = []
        if detections:
            depth = self.client.estimate_depth(obs)
            for detection in detections[:self.max_candidates_per_frame]:
                if detection["confidence"] < self.detector.cfg.confidence_threshold:
                    continue
                try:
                    position = self.position(obs, depth, detection["box"])
                except TargetGeometryError as exc:
                    self.event("depth_rejected", frame_id=obs.frame_id, reason=str(exc))
                    continue
                candidates.append({"box": np.asarray(detection["box"]).tolist(),
                                   "confidence": float(detection["confidence"]),
                                   "position_cm": position.tolist()})
        self.event("detection", frame_id=obs.frame_id, timestamp_s=obs.timestamp_s,
                   capture_pose=obs.pose, candidates=candidates,
                   calibration_quality=obs.calibration_quality,
                   geometry_assumptions=obs.geometry_assumptions,
                   depth_source="box_core", inference_s=time.monotonic()-start)
        return candidates

    def _next_target(self, segment):
        while True:
            result = self.pipeline.pop()
            if result is None:
                return None
            result_segment, obs, candidates = result
            age = obs.age_s + time.monotonic() - obs.received_monotonic
            if result_segment != segment or age > self.patrol["max_detection_age_s"]:
                self.event("result_discarded", frame_id=obs.frame_id, age_s=age)
                continue
            hit = self._claim_candidates(obs, candidates)
            if hit:
                return hit

    def _claim_candidates(self, obs, candidates):
        age = obs.age_s + time.monotonic() - obs.received_monotonic
        if age > self.patrol["max_detection_age_s"]:
            self.event("result_discarded", frame_id=obs.frame_id, age_s=age)
            return None
        for candidate in candidates:
            record = self.memory.claim(candidate["position_cm"])
            if record:
                self.event("target_claimed", target_id=record.target_id,
                           frame_id=obs.frame_id, capture_pose=obs.pose, age_s=age)
                return obs, candidate, record
            self.event("duplicate", frame_id=obs.frame_id,
                       position_cm=candidate["position_cm"])
        return None

    def _start_navigation(self, target):
        self._ensure_flight_safety("before navigation")
        if self.active_navigation is not None:
            raise v20.FlightSafetyError("Attempted overlapping navigation")
        if target["z"] < self.safe_z_cm:
            raise v20.FlightSafetyError("Navigation target below safe world height")
        try:
            task_id = self.client.navigate(target)
            if not isinstance(task_id, str) or not task_id:
                raise ValueError("Invalid task_id")
        except Exception as exc:
            raise v20.FlightSafetyError(f"Navigation submit failed: {exc}") from exc
        self.active_navigation = task_id
        self.event("navigation_started", task_id=task_id, target=target)
        return task_id

    def _navigation_state(self, task_id):
        try:
            state = self.client.navigation_status(task_id)
            if state["status"] == "failed":
                raise RuntimeError(state.get("error", "navigation failed"))
            if state["status"] in {"arrived", "cancelled"}:
                if state.get("stopped") is not True:
                    raise RuntimeError("Terminal navigation lacks stopped confirmation")
            return state["status"]
        except Exception as exc:
            raise v20.FlightSafetyError(f"Navigation status failed: {exc}") from exc

    def _cancel_navigation(self):
        if self.active_navigation is None:
            return
        task_id = self.active_navigation
        self.client.cancel_navigation(task_id)
        deadline = time.monotonic() + self.patrol["cancel_timeout_s"]
        while time.monotonic() < deadline:
            self._ensure_flight_safety("waiting for cancellation")
            state = self._navigation_state(task_id)
            if state in {"cancelled", "arrived"}:
                self.active_navigation = None
                self.event("navigation_stopped", task_id=task_id, status=state)
                return
            time.sleep(self.patrol["poll_interval_s"])
        raise v20.FlightSafetyError("Navigation cancellation/stop confirmation timed out")

    def _navigate_to_world_pose(self, target_pose, context):
        self.set_phase(context)
        task_id = self._start_navigation(target_pose)
        deadline = time.monotonic() + self.patrol["navigation_timeout_s"]
        while time.monotonic() < deadline:
            self._ensure_flight_safety(context)
            state = self._navigation_state(task_id)
            if state == "arrived":
                self.active_navigation = None
                return self._verify_arrival(target_pose)
            if state == "cancelled":
                raise v20.FlightSafetyError("Navigation cancelled unexpectedly")
            time.sleep(self.patrol["poll_interval_s"])
        self._cancel_navigation()
        raise v20.FlightSafetyError(f"Navigation timed out: {context}")

    def _verify_arrival(self, target):
        actual = self._get_pose_for_flight("verify arrival")
        distance = np.linalg.norm([actual[k]-target[k] for k in ("x", "y", "z")])
        yaw_error = abs(self._normalize_angle_deg(actual["yaw"]-target["yaw"]))
        if distance > self.position_tolerance_cm or yaw_error > self.yaw_tolerance_deg:
            raise v20.FlightSafetyError(f"Arrival outside tolerances: {distance}cm/{yaw_error}deg")
        return actual

    def _reacquire(self, record):
        for _ in range(self.patrol["reacquire_attempts"]):
            obs = self.client.observe()
            candidates = self.infer_observation(obs)
            matches = sorted(candidates, key=lambda c: np.linalg.norm(
                np.asarray(c["position_cm"])-record.position_cm))
            if matches and np.linalg.norm(np.asarray(matches[0]["position_cm"])-record.position_cm) < self.memory.distance_cm:
                box = np.asarray(matches[0]["box"])
                self.tracker.reset()
                bbox, mask = self.tracker.track_with_mask(obs.rgb, box=box)
                if not np.any(mask):
                    continue
                # Refine box-based identity with foreground depth before TRACK.
                depth = self.client.estimate_depth(obs)
                try:
                    refined = self.position(obs, depth, bbox, mask)
                except TargetGeometryError:
                    continue
                if np.linalg.norm(refined-record.position_cm) >= self.memory.distance_cm:
                    continue
                self.event("reacquired", target_id=record.target_id,
                           frame_id=obs.frame_id, position_cm=refined.tolist())
                self.exec_rotate_action_deg(
                    self.prepare_rotate_action_deg(self.get_bbox_state(bbox)["horizontal_offset"]),
                    context="v21 initial target alignment")
                return True
        return False

    def _visit_target(self, hit):
        obs, candidate, record = hit
        self.pipeline.pause()
        self.set_phase("STOPPING", target_id=record.target_id)
        self._cancel_navigation()
        # Cancel flight first; then wait for GPU workers before SAM2/main use.
        self.pipeline.wait_idle(self.patrol["worker_idle_timeout_s"])
        target_dir = self._mission_vis_dir / f"target_{record.target_id:03d}_attempt_{record.attempts:02d}"
        target_dir.mkdir(parents=True, exist_ok=True)
        self.vis_dir = str(target_dir)
        self.tracker.set_vis_dir(self.vis_dir)
        cv2.imwrite(str(target_dir / "trigger.jpg"), cv2.cvtColor(obs.rgb, cv2.COLOR_RGB2BGR))
        (target_dir / "trigger.json").write_text(json.dumps({
            "frame_id": obs.frame_id, "timestamp_s": obs.timestamp_s, "pose": obs.pose,
            "candidate": candidate, "intrinsics": obs.intrinsics.tolist(),
            "world_from_camera_optical_cm": obs.world_from_camera.tolist(),
            "localization_epoch": obs.localization_epoch,
            "rectified": obs.rectified, "calibration_quality": obs.calibration_quality,
            "geometry_assumptions": obs.geometry_assumptions,
        }, indent=2), encoding="utf-8")
        self._navigate_to_world_pose(obs.pose, "RETURN_TO_CAPTURE")
        success, refined = False, None
        self.last_track_observation = None
        try:
            self.set_phase("REACQUIRE", target_id=record.target_id)
            if self._run_task_stage("reacquire", lambda: self._reacquire(record)):
                self.set_phase("TRACK", target_id=record.target_id)
                success = self._run_task_stage("track", self.track)
                if success and self.last_track_observation:
                    last = self.last_track_observation
                    try:
                        refined = self.position(self.capture_observation, last["depth_raw"],
                                                last["bbox"], last["mask"])
                        if np.linalg.norm(refined-record.position_cm) >= self.memory.distance_cm:
                            self.event("identity_uncertain", target_id=record.target_id,
                                       position_cm=refined.tolist())
                            success, refined = False, None
                    except TargetGeometryError as exc:
                        self.event("completion_geometry_failed", reason=str(exc))
                        success = False
        except v20.TaskFailure as exc:
            self.event("target_failed", target_id=record.target_id, reason=str(exc))
        self.memory.finish(record, success, refined)
        self.event("target_finished", target_id=record.target_id, success=success,
                   targets=self.memory.snapshot())
        # No SCAN or waypoint SEARCH; return to the synchronized trigger pose.
        self._navigate_to_world_pose(obs.pose, "RETURN_TO_ROUTE")
        return success

    def _patrol_segment(self, index, waypoint, target):
        while True:
            self.set_phase("PATROL", segment=index, waypoint=waypoint["name"])
            self.pipeline.resume(index)
            task_id = self._start_navigation(target)
            deadline = time.monotonic() + self.patrol["navigation_timeout_s"]
            hit = None
            while time.monotonic() < deadline:
                self._ensure_flight_safety("patrol")
                hit = self._next_target(index)
                if hit:
                    break
                state = self._navigation_state(task_id)
                if state == "cancelled":
                    raise v20.FlightSafetyError("Patrol navigation cancelled externally")
                if state == "arrived":
                    self.active_navigation = None
                    self._verify_arrival(target)
                    self.pipeline.pause(drain=True)
                    self.pipeline.wait_idle(self.patrol["worker_idle_timeout_s"])
                    hit = self._next_target(index)
                    if not hit:
                        # Even an immediately satisfied waypoint gets a final view.
                        final_obs = self.client.observe()
                        hit = self._claim_candidates(final_obs, self.infer_observation(final_obs))
                    if not hit:
                        self.pipeline.pause()
                        self.event("waypoint_arrived", segment=index, name=waypoint["name"])
                        return True
                    break
                time.sleep(self.patrol["poll_interval_s"])
            else:
                self.pipeline.pause()
                self._cancel_navigation()
                raise v20.FlightSafetyError("Patrol navigation timed out")
            success = self._visit_target(hit)
            if not success and self.task_failure_policy == "return_home":
                return False

    def run_mission(self, detector_prompt):
        self.detector.set_prompt(detector_prompt)
        self.pipeline = PerceptionPipeline(self.client.observe, self.infer_observation,
                                           capture_fps=self.patrol["capture_fps"],
                                           det_interval=self.patrol["det_interval"])
        try:
            for index, waypoint in enumerate(self.mission_waypoints):
                if not self._patrol_segment(index, waypoint, self._mission_to_world_pose(waypoint)):
                    break
            self.pipeline.pause()
            self.pipeline.wait_idle(self.patrol["worker_idle_timeout_s"])
            self._navigate_to_world_pose(self.mission_origin_pose, "RETURN_HOME")
            self.event("mission_finished", targets=self.memory.snapshot(),
                       perception_stats=dict(self.pipeline.stats))
            return self.memory.snapshot()
        finally:
            self.pipeline.pause()
            try:
                if self.active_navigation is not None:
                    self._cancel_navigation()
            finally:
                self.pipeline.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", "--client", default="owl_ego", choices=["owl_ego"])
    parser.add_argument("--config", default="owl/v21.yaml")
    parser.add_argument("--det", choices=["sam3"], default="sam3")
    parser.add_argument("--trk", choices=["sam2"], default="sam2")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--img")
    group.add_argument("--text")
    parser.add_argument("--box")
    parser.add_argument("--server-host", default="127.0.0.1")
    parser.add_argument("--server-port", type=int, default=8765)
    parser.add_argument("--http-timeout-s", type=float)
    parser.add_argument("--det-interval", type=int)
    parser.add_argument("--exp_name", default=str(Path(__file__).resolve().parents[3]/"logs"/"v21"))
    args = parser.parse_args()
    try:
        prompt = v20._build_detector_prompt(args)
        config = v20._load_config(v20._resolve_config_path(args.config, "owl"))
        if args.det_interval is not None:
            config["patrol"]["det_interval"] = args.det_interval
        patrol = validate_patrol_config(config)
        connection = config.get("owl_ego", {})
        if not isinstance(connection, dict) or set(connection) - {
                "allow_approximate_geometry", "auto_arm", "stop_timeout_s", "observation_retry_s"}:
            raise ValueError("Invalid owl_ego connection configuration")
        timeout = (config["runtime"]["http_request_timeout_s"]
                   if args.http_timeout_s is None else args.http_timeout_s)
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("HTTP timeout must be finite and positive")
    except (ValueError, KeyError) as exc:
        parser.error(str(exc))
    directory = v20._timestamped_experiment_dir(args.exp_name, dt.datetime.now().strftime("%Y%m%d_%H%M%S"))
    (directory/"vis").mkdir(parents=True, exist_ok=True)
    logger = v20._configure_logger(directory/"log.txt")
    (directory/"config.json").write_text(json.dumps(config, indent=2), encoding="utf-8")
    client = OwlEgoClient(host=args.server_host, port=args.server_port, timeout_s=timeout,
                          observation_max_age_s=patrol["observation_max_age_s"],
                          observation_max_sync_error_s=patrol["observation_max_sync_error_s"],
                          **connection)
    # Load models and validate inherited TRACK configuration before taking control.
    agent = PatrolAgent(client=client, detector=v20.build_detector(args.det, config),
                        tracker=v20.build_tracker(args.trk, config, str(directory/"vis")),
                        config=config, detector_name=args.det, tracker_name=args.trk,
                        vis_dir=str(directory/"vis"), logger=logger)
    mission_error = None
    try:
        agent.connect()
        agent.run_mission(prompt)
    except BaseException as exc:
        mission_error = exc
        logger.exception("v21 mission aborted")
        raise
    finally:
        cleanup_error = None
        try:
            if client.session_id:
                if agent.active_navigation:
                    try:
                        agent._cancel_navigation()
                    except Exception:
                        logger.exception("Navigation cancellation failed before landing")
                v20._land_after_task(client, logger)
        except Exception as exc:
            cleanup_error = exc
            logger.exception("Landing cleanup failed; landing is not confirmed")
        finally:
            try:
                client.close()
            except Exception as exc:
                cleanup_error = cleanup_error or exc
                logger.exception("Session release failed")
        if mission_error is None and cleanup_error is not None:
            raise cleanup_error


if __name__ == "__main__":
    main()
