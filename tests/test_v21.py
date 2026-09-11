"""Offline tests: no model weights, ROS, drone connection or flight commands."""
import base64
import copy
import csv
from dataclasses import replace
import json
import math
import os
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"src"))
import cv2
import numpy as np
import yaml

from agent.tjk.patrol import PerceptionPipeline, TargetGeometryError, TargetMemory, target_position
from agent.tjk.v21 import PatrolAgent, validate_patrol_config
from agent.tjk.v20 import FlightSafetyError
from robot_client.owl_ego import Observation, OwlEgoClient, decode_observation
from robot_client.base import BaseClient, RobotHTTPError


ROOT = Path(__file__).resolve().parents[1]


def observation(frame="1", pose=None):
    # Optical forward corresponds to ENU X, optical right to ENU -Y.
    t = np.array([[0,0,1,10], [-1,0,0,20], [0,-1,0,100], [0,0,0,1]], float)
    return Observation(frame, 1234.5, time.monotonic(), 0.01,
                       pose or dict(x=10., y=-20., z=100., yaw=0.),
                       np.zeros((20,20,3), np.uint8),
                       np.array([[10,0,10],[0,10,10],[0,0,1]], float),
                       t, "epoch-a", "odom")


def wire_observation():
    obs = observation()
    ok, jpg = cv2.imencode(".jpg", obs.rgb)
    assert ok
    return dict(ok=True, frame_id=obs.frame_id, timestamp_s=obs.timestamp_s,
                age_s=0.01, sync_error_s=0.01, pose=obs.pose, image_size=[20,20],
                rectified=True, calibration_quality="calibrated",
                rgb_jpeg_base64=base64.b64encode(jpg).decode(),
                intrinsics=obs.intrinsics.tolist(),
                world_from_camera_optical_cm=obs.world_from_camera.tolist(),
                localization_epoch="epoch-a", world_frame="odom")


class GeometryTests(unittest.TestCase):
    def test_camera_optical_to_public_world_with_translation(self):
        obs = observation()
        depth = np.full((20,20), 200.)
        mask = np.zeros((20,20), bool)
        mask[9:12, 9:12] = True
        p = target_position(obs, depth, [8,8,13,13], mask=mask, min_pixels=9)
        np.testing.assert_allclose(p, [210,-20,100])

    def test_rotation_and_nonzero_pixel_offset(self):
        obs = observation()
        depth = np.full((20,20), 200.)
        mask = np.zeros((20,20), bool)
        mask[11:14, 12:15] = True
        p = target_position(obs, depth, [11,10,15,15], mask=mask, min_pixels=9)
        np.testing.assert_allclose(p, [210,40,60])

    def test_mask_rejects_background_depth(self):
        obs = observation()
        depth = np.full((20,20), 1000.)
        depth[9:12,9:12] = 200
        mask = np.zeros((20,20), bool)
        mask[9:12,9:12] = True
        p = target_position(obs, depth, [2,2,18,18], mask=mask, min_pixels=9)
        np.testing.assert_allclose(p, [210,-20,100])

    def test_invalid_and_disperse_depth(self):
        for depth in (np.zeros((20,20)), np.full((20,20), np.nan),
                      np.tile([10.,1000.], (20,10))):
            with self.assertRaises(TargetGeometryError):
                target_position(observation(), depth, [2,2,18,18])

    def test_same_world_object_from_two_camera_positions(self):
        a = observation()
        b_t = a.world_from_camera.copy()
        b_t[0,3] += 100
        b = replace(a, world_from_camera=b_t)
        mask = np.zeros((20,20), bool)
        mask[9:12,9:12] = True
        pa = target_position(a, np.full((20,20),200.), [8,8,13,13], mask=mask,min_pixels=9)
        pb = target_position(b, np.full((20,20),100.), [8,8,13,13], mask=mask,min_pixels=9)
        np.testing.assert_allclose(pa,pb)


class MemoryTests(unittest.TestCase):
    def test_pending_completed_and_distinct_targets(self):
        memory = TargetMemory(100)
        record = memory.claim([0,0,100], now=0)
        self.assertIsNone(memory.claim([90,0,100], now=1))
        memory.finish(record, True)
        self.assertIsNone(memory.claim([10,0,100], now=100))
        self.assertIsNotNone(memory.claim([101,0,100], now=100))

    def test_failed_retry_cooldown_and_limit(self):
        memory = TargetMemory(100,30,2)
        r = memory.claim([0,0,100], now=0)
        memory.finish(r,False,now=1)
        self.assertIsNone(memory.claim([0,0,100],now=30))
        self.assertIs(memory.claim([0,0,100],now=31),r)
        memory.finish(r,False,now=32)
        self.assertIsNone(memory.claim([0,0,100],now=100))


class ObservationTests(unittest.TestCase):
    def test_wire_roundtrip(self):
        obs = decode_observation(wire_observation())
        self.assertEqual(obs.rgb.shape,(20,20,3))
        self.assertEqual(obs.pose["y"],-20)

    def test_bad_calibration_sync_and_staleness(self):
        for key,value in (("age_s",1), ("sync_error_s",0.1), ("rectified",False),
                          ("intrinsics",np.eye(2).tolist()), ("image_size",[1,1])):
            data = wire_observation()
            data[key] = value
            with self.assertRaises(ValueError):
                decode_observation(data)

    def test_epoch_reset_fails(self):
        client = OwlEgoClient()
        client.rpc = Mock(return_value=wire_observation())
        client.observe()
        data = wire_observation()
        data["localization_epoch"]="epoch-b"
        client.rpc.return_value=data
        with self.assertRaisesRegex(RuntimeError,"epoch"):
            client.observe()

    def test_task_identity_and_lease_failure(self):
        client = OwlEgoClient()
        client.rpc = Mock(return_value=dict(ok=True,task_id="wrong",status="arrived"))
        with self.assertRaisesRegex(RuntimeError,"task_id"):
            client.navigation_status("wanted")
        client._lease_error = RuntimeError("network lost")
        with self.assertRaisesRegex(RuntimeError,"lease"):
            client.observe()


class HttpContractTests(unittest.TestCase):
    def test_real_local_http_session_navigation_and_blocking_track_payload(self):
        calls=[]
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*args): pass
            def do_GET(self): self.handle_rpc()
            def do_POST(self): self.handle_rpc()
            def handle_rpc(self):
                payload=json.loads(self.rfile.read(int(self.headers.get("Content-Length",0))) or b'{}')
                calls.append((self.command,self.path,payload))
                if self.path=="/v21/capabilities":
                    result=dict(backend="owl_ego",protocol_version=1,async_navigation=True,
                                cancel_and_hold=True,synchronized_observation=True,
                                control_lease=True,relative_xyz_yaw=True)
                elif self.path=="/v21/observation": result=wire_observation()
                elif self.path=="/v21/session": result=dict(session_id="session-1")
                elif self.path=="/health":
                    result=FakeClient().health()
                elif self.path=="/motion_tolerances":
                    result=dict(motion_tolerances=dict(position_tolerance_cm=15,yaw_tolerance_deg=5,
                                position_error_metric="euclidean_3d",source="owl_ego"))
                elif self.path=="/v21/navigation": result=dict(task_id="nav-1")
                elif self.path in ("/move_relative_xyz_yaw", "/land"):
                    result=dict(task_id="motion-1")
                elif self.path.startswith("/v21/navigation/status"):
                    result=(dict(task_id="motion-1",status="arrived",stopped=True)
                            if "motion-1" in self.path else
                            dict(task_id="nav-1",status="cancelled",stopped=True))
                else: result={}
                raw=json.dumps(dict(ok=True,**{k:v for k,v in result.items() if k!="ok"})).encode()
                self.send_response(200)
                self.send_header("Content-Type","application/json")
                self.send_header("Content-Length",str(len(raw)))
                self.end_headers()
                self.wfile.write(raw)
        server=ThreadingHTTPServer(("127.0.0.1",0),Handler)
        thread=threading.Thread(target=server.serve_forever,daemon=True)
        thread.start()
        client=OwlEgoClient(port=server.server_port)
        client.depth_service=Mock()
        try:
            client.start()
            self.assertEqual(client.session_id,"session-1")
            task=client.navigate(dict(x=0,y=100,z=100,yaw=0))
            client.cancel_navigation(task)
            self.assertEqual(client.navigation_status(task)["status"],"cancelled")
            client.move_rel_xyz_yaw(x=20,y=0,z=0,yaw=5,timeout_s=15)
            time.sleep(.55)
            client.land()
        finally:
            client.close()
            server.shutdown()
            server.server_close()
            thread.join(2)
        paths=[p for _,p,_ in calls]
        self.assertNotIn("/takeoff",paths)  # already airborne
        self.assertIn("/v21/heartbeat",paths)
        movement=next(b for _,p,b in calls if p=="/move_relative_xyz_yaw")
        self.assertEqual(movement["session_id"],"session-1")
        self.assertEqual(movement["timeout_s"],15)
        self.assertTrue(movement["request_id"])

    def test_old_backend_fails_before_session_or_takeoff(self):
        client=OwlEgoClient()
        client.depth_service=Mock()
        client.rpc=Mock(return_value=dict(ok=True,backend="owl",protocol_version=1))
        with self.assertRaisesRegex(RuntimeError,"owl_ego"):
            client.start()
        self.assertIsNone(client.session_id)
        self.assertEqual(client.rpc.call_count,1)


class PipelineTests(unittest.TestCase):
    def wait_until(self,predicate):
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            if predicate(): return
            time.sleep(.005)
        self.fail("worker condition timed out")

    def test_latest_frame_queue_and_generation_fence(self):
        count=[0]
        entered=threading.Event()
        release=threading.Event()
        seen=[]
        def capture():
            count[0]+=1
            return observation(str(count[0]))
        def infer(obs):
            seen.append(obs.frame_id)
            entered.set()
            release.wait(2)
            return ["hit"]
        pipeline=PerceptionPipeline(capture,infer,capture_fps=200)
        try:
            pipeline.resume(0)
            self.assertTrue(entered.wait(2))
            self.wait_until(lambda:count[0]>=6)
            self.assertGreater(pipeline.stats["overwritten"],0)
            pipeline.pause()
            release.set()
            pipeline.wait_idle(2)
            self.assertIsNone(pipeline.pop())
            self.assertEqual(len(seen),1)
            self.assertGreater(pipeline.stats["stale_results"],0)
            pipeline.resume(1)
            self.wait_until(lambda:bool(pipeline.results))
            pipeline.pause(drain=True)
            pipeline.wait_idle(2)
            self.assertEqual(pipeline.pop()[0],1)
        finally:
            release.set()
            pipeline.close()

    def test_interval_counts_unique_exposures(self):
        count=[0]
        frames=[]
        def capture():
            count[0]+=1
            return observation(str((count[0]+1)//2))
        pipeline=PerceptionPipeline(capture,lambda obs:frames.append(int(obs.frame_id)),
                                    capture_fps=100,det_interval=3)
        try:
            pipeline.resume(0)
            self.wait_until(lambda:len(frames)>=3)
            pipeline.pause(drain=True)
            pipeline.wait_idle(2)
            self.assertEqual(frames[:3],[1,4,7])
        finally:
            pipeline.close()

    def test_worker_failure_surfaces_to_main(self):
        def fail(obs): raise ValueError("model failed")
        pipeline=PerceptionPipeline(lambda:observation(),fail)
        try:
            pipeline.resume(0)
            self.wait_until(lambda:pipeline.error is not None)
            with self.assertRaisesRegex(RuntimeError,"model failed"):
                pipeline.pop()
        finally:
            pipeline.close()


class FakeClient(OwlEgoClient):
    def __init__(self):
        super().__init__()
        self.pose=dict(x=0.,y=0.,z=100.,yaw=0.)
        self.last_observation=None
        self._frame_identity=("odom","epoch-a")
        self.tasks={}
        self.calls=[]
        self.counter=0
        self.frame=0
        self.base_url="offline"
    def check_lease(self): pass
    def health(self):
        return dict(ok=True,health=dict(initialized=True,airborne=True,control_ready=True,
                    hold_ready=True, stopped=True, active_task_id=None, planner_state="not_required",
                    odom_ok=True,rgb_ok=True,planner_ok=False,localization_epoch="epoch-a"))
    def get_pose(self): return self.pose.copy()
    def observe(self):
        self.frame+=1
        return observation(str(self.frame),self.pose.copy())
    def estimate_depth(self,obs): return np.full((20,20),200.)
    def navigate(self,pose):
        self.counter+=1
        task=str(self.counter)
        self.tasks[task]=dict(pose=pose.copy(),status="executing",polls=0)
        self.calls.append(("navigate",pose.copy()))
        return task
    def navigation_status(self,task):
        record=self.tasks[task]
        record["polls"]+=1
        if record["status"]=="executing" and record["polls"]>=3:
            record["status"]="arrived"
            self.pose=record["pose"].copy()
        return dict(ok=True,task_id=task,status=record["status"],
                    stopped=record["status"] in {"arrived","cancelled"})
    def cancel_navigation(self,task):
        self.calls.append(("cancel",task))
        self.tasks[task]["status"]="cancelled"


class MissionTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        config=yaml.safe_load((ROOT/"src/agent/config/owl/v21.yaml").read_text())
        # Test route is independent of the operator's current flight YAML.
        config["mission"]["waypoints"]=[
            dict(name="test_B",x_cm=0.,y_cm=100.,z_cm=0.,yaw_deg=0.),
            dict(name="test_C",x_cm=0.,y_cm=-150.,z_cm=0.,yaw_deg=0.)]
        config["patrol"].update(capture_fps=60.,poll_interval_s=.02)
        self.config=config
        self.client=FakeClient()
        detector=Mock()
        detector.cfg.confidence_threshold=.5
        self.agent=PatrolAgent(config=config,client=self.client,detector=detector,
                               tracker=Mock(),detector_name="sam3",tracker_name="sam2",
                               vis_dir=str(Path(self.temp.name)/"vis"),save_vis=False)
        self.agent.event=Mock()
        self.agent._csv_pose=Mock(return_value={})
        self.client.event=self.agent.event
        self.agent.mission_origin_pose=self.client.pose.copy()
        self.agent.position_tolerance_cm=15
        self.agent.yaw_tolerance_deg=5
        self.agent._reacquire=Mock(return_value=True)
        self.agent.track=Mock(return_value=True)
    def tearDown(self): self.temp.cleanup()

    def test_full_route_target_return_dedup_and_home(self):
        # Always sees the same object; completed memory must prevent retrigger.
        self.agent.infer_observation=lambda obs:[dict(box=[4,4,16,16],confidence=.9,
                                                    position_cm=[210,-20,100])]
        records=self.agent.run_mission("bottle")
        self.assertEqual(len(records),1)
        self.assertEqual(records[0]["status"],"completed")
        self.agent.track.assert_called_once()
        self.assertTrue(any(c[0]=="cancel" for c in self.client.calls))
        goals=[c[1] for c in self.client.calls if c[0]=="navigate"]
        self.assertIn(dict(x=0.,y=100.,z=100.,yaw=0.),goals)
        self.assertIn(dict(x=0.,y=-150.,z=100.,yaw=0.),goals)
        self.assertEqual(goals[-1],self.agent.mission_origin_pose)
        # The two detour return goals use exposure pose, not a later pose.
        trigger=self.agent.event.call_args_list
        claimed=next(c.kwargs["capture_pose"] for c in trigger if c.args[0]=="target_claimed")
        self.assertEqual(goals[1:3],[claimed,claimed])

    def test_failure_returns_then_continues(self):
        self.agent.infer_observation=lambda obs:[dict(box=[4,4,16,16],confidence=.9,
                                                    position_cm=[210,-20,100])]
        self.agent._reacquire.return_value=False
        records=self.agent.run_mission("bottle")
        self.assertEqual(records[0]["status"],"failed")
        self.agent.track.assert_not_called()
        self.assertTrue(any(c[0]=="navigate" and c[1]["y"]==-150 for c in self.client.calls))

    def test_failure_policy_returns_home_early(self):
        self.agent.task_failure_policy="return_home"
        self.agent.infer_observation=lambda obs:[dict(box=[4,4,16,16],confidence=.9,
                                                    position_cm=[210,-20,100])]
        self.agent._reacquire.return_value=False
        self.agent.run_mission("bottle")
        self.assertFalse(any(c[0]=="navigate" and c[1]["y"]==-150 for c in self.client.calls))
        self.assertEqual(self.client.calls[-1],("navigate",self.agent.mission_origin_pose))

    def test_no_target_visits_entire_route(self):
        self.agent.infer_observation=lambda obs:[]
        self.assertEqual(self.agent.run_mission("bottle"),[])
        self.assertEqual([c[1]["y"] for c in self.client.calls if c[0]=="navigate"],[100,-150,0])

    def test_arrival_requires_stop_and_pose_confirmation(self):
        self.client.navigation_status=Mock(return_value=dict(status="arrived",stopped=False))
        with self.assertRaises(FlightSafetyError): self.agent._navigation_state("1")
        with self.assertRaises(FlightSafetyError):
            self.agent._verify_arrival(dict(x=1000,y=0,z=100,yaw=0))

    def test_config_rejects_legacy_semantics_and_nonfinite_values(self):
        for key,value in (("capture_fps",float('nan')), ("det_interval",0), ("det_interval",1.5)):
            config=copy.deepcopy(self.config)
            config["patrol"][key]=value
            with self.assertRaises(ValueError): validate_patrol_config(config)
        config=copy.deepcopy(self.config)
        config["mission"]["waypoints"][0]["only_arrive"]=True
        with self.assertRaises(ValueError): validate_patrol_config(config)

    def test_cancel_arrival_race_and_failed_task_are_not_confused(self):
        self.agent.active_navigation="a"
        self.client.cancel_navigation=Mock()
        self.client.navigation_status=Mock(return_value=dict(status="arrived",stopped=True))
        self.agent._cancel_navigation()
        self.assertIsNone(self.agent.active_navigation)
        self.agent.active_navigation="b"
        self.client.navigation_status.return_value=dict(status="failed",stopped=True,error="planner lost")
        with self.assertRaisesRegex(FlightSafetyError,"planner lost"):
            self.agent._cancel_navigation()
        self.assertEqual(self.agent.active_navigation,"b")


def approximate_wire():
    data = wire_observation()
    data.update(rectified=False, calibration_quality="approximate", geometry_assumptions=dict(
        intrinsics="approximate_fov", assumed_horizontal_fov_deg=90.,
        principal_point="image_center", square_pixels_assumed=True,
        distortion="unknown_not_corrected", extrinsics="body_coincident_fixed",
        camera_translation="body_coincident_assumption"))
    return data


class CurrentContractTests(unittest.TestCase):
    def client(self, **kwargs):
        client = OwlEgoClient(**kwargs)
        client._frame_identity = ("odom", "epoch-a")
        client.health = Mock(return_value=FakeClient().health())
        return client

    def test_approximate_opt_in_and_metadata(self):
        data = approximate_wire()
        with self.assertRaises(ValueError): decode_observation(data)
        obs = decode_observation(data, allow_approximate_geometry=True)
        self.assertFalse(obs.rectified)
        self.assertEqual(obs.geometry_assumptions["assumed_horizontal_fov_deg"], 90)
        for key, value in (("geometry_assumptions", {}), ("calibration_quality", "unknown"),
                           ("age_s", 1), ("sync_error_s", .2)):
            bad = copy.deepcopy(data); bad[key] = value
            with self.assertRaises(ValueError):
                decode_observation(bad, allow_approximate_geometry=True)
        data["intrinsics"][0][0] = 15
        with self.assertRaisesRegex(ValueError, "contradict"):
            decode_observation(data, allow_approximate_geometry=True)

    def test_rectified_approximate_extrinsics_still_require_opt_in(self):
        data = approximate_wire()
        data["rectified"] = True
        data["geometry_assumptions"].update(intrinsics="camera_info", distortion="corrected")
        with self.assertRaises(ValueError): decode_observation(data)
        self.assertEqual(decode_observation(data, allow_approximate_geometry=True).calibration_quality,
                         "approximate")

    def test_only_explicit_temporary_observation_is_retried(self):
        transient = RobotHTTPError(503, "/v21/observation", json.dumps(dict(
            error_code="observation_unavailable", retryable=True)))
        client = self.client(observation_retry_s=.12)
        client.rpc = Mock(side_effect=[transient, wire_observation()])
        client.observe(); self.assertEqual(client.rpc.call_count, 2)
        for status, code, retryable in [(409,"localization_epoch_changed",False),
                                        (422,"invalid_observation",False), (503,"other",True)]:
            client.rpc = Mock(side_effect=RobotHTTPError(status,"/v21/observation",json.dumps(
                dict(error_code=code,retryable=retryable))))
            with self.assertRaises(RobotHTTPError): client.observe()
            self.assertEqual(client.rpc.call_count, 1)
        client.rpc = Mock(side_effect=transient)
        start = time.monotonic()
        with self.assertRaises(RobotHTTPError): client.observe()
        self.assertLess(time.monotonic()-start, .2)

    def test_phase_health_and_persistent_rgb_absence(self):
        client = self.client(observation_retry_s=.01)
        h = client.health.return_value["health"]
        client.flight_health()  # not_required, planner_ok false
        h.update(planner_state="starting",active_task_id="a")
        client.navigation_status = Mock(return_value=dict(status="planning"))
        client.flight_health()
        h.update(rgb_ok=False,observation_error_code="observation_unavailable",observation_retryable=True)
        client.flight_health()
        time.sleep(.015)
        with self.assertRaisesRegex(RuntimeError,"retry budget"): client.flight_health()
        h.update(rgb_ok=True,planner_state="lost")
        with self.assertRaisesRegex(RuntimeError,"Planner"): client.flight_health()
        h.update(planner_state="not_required",localization_epoch="new")
        with self.assertRaisesRegex(RuntimeError,"reset"): client.flight_health()

    def test_current_stop_required_and_timeout_has_no_submission(self):
        client = self.client(stop_timeout_s=.06)
        h = client.health.return_value["health"]
        h["stopped"] = False
        client.rpc = Mock()
        with self.assertRaisesRegex(RuntimeError,"stop confirmation"):
            client.navigate(dict(x=1,y=0,z=100,yaw=0))
        client.rpc.assert_not_called()
        first = copy.deepcopy(h)
        h["stopped"] = True
        client.health.side_effect = [dict(ok=True,health=first),dict(ok=True,health=h)]
        client.rpc.return_value = dict(ok=True,task_id="b")
        self.assertEqual(client.navigate(dict(x=1,y=0,z=100,yaw=0)),"b")

    def test_startup_budget_and_ready_to_terminal_race(self):
        client=self.client()
        client.health.return_value["health"].update(planner_state="starting",active_task_id="a")
        client.navigation_status=Mock(return_value=dict(status="arrived",stopped=True))
        client.flight_health()  # Later task snapshot can already be terminal.
        client._starting_since=time.monotonic()-11
        with self.assertRaisesRegex(RuntimeError,"10 s"): client.flight_health()

    def test_relative_monitors_epoch_and_cancels_only_owned_task(self):
        client=self.client()
        client.wait_stopped=Mock()
        healthy=client.health.return_value["health"]
        healthy["active_task_id"]="relative-a"
        reset={**healthy,"localization_epoch":"new"}
        client.health.side_effect=[dict(ok=True,health=healthy),dict(ok=True,health=reset)]
        client.get_pose=Mock(return_value=dict(x=0,y=0,z=100,yaw=0))
        release=threading.Event();completed=threading.Event()
        def block(*args,**kwargs):
            release.wait(2);completed.set()
            return dict(ok=True)
        client.cancel_navigation=Mock(side_effect=lambda task:release.set())
        try:
            with patch.object(BaseClient,"move_rel_xyz_yaw",side_effect=block):
                with self.assertRaisesRegex(RuntimeError,"reset"):
                    client.move_rel_xyz_yaw(x=20,z=0)
                self.assertTrue(completed.wait(1))
            client.cancel_navigation.assert_called_once_with("relative-a")
        finally: release.set()

    def test_relative_heartbeat_failure_is_fatal(self):
        client=self.client();client.wait_stopped=Mock()
        release=threading.Event()
        def block(*args,**kwargs):
            client._lease_error=RuntimeError("heartbeat timeout")
            release.wait(2);return dict(ok=True)
        try:
            with patch.object(BaseClient,"move_rel_xyz_yaw",side_effect=block):
                with self.assertRaisesRegex(RuntimeError,"lease"):
                    client.move_rel_xyz_yaw(x=20)
        finally: release.set()

    def test_readiness_409_is_not_retried_as_new_navigation(self):
        client=self.client();client.wait_stopped=Mock()
        client.rpc=Mock(side_effect=RobotHTTPError(409,"/v21/navigation","busy"))
        with self.assertRaises(RobotHTTPError):
            client.navigate(dict(x=20,y=0,z=100,yaw=0))
        client.rpc.assert_called_once()

    def test_mutation_error_preserves_exact_request_for_replay(self):
        client = self.client(); client.session_id = "s"
        with patch.object(BaseClient,"_request_json",side_effect=RuntimeError("uncertain")) as call:
            with self.assertRaises(RuntimeError) as cm:
                client._request_json("POST","/move_relative_xyz_yaw",dict(x=20,y=0,z=0,yaw=0))
            body = cm.exception.request_payload
            with self.assertRaises(RuntimeError):
                client._request_json("POST","/move_relative_xyz_yaw",body)
            self.assertEqual(call.call_args_list[0],call.call_args_list[1])
            self.assertTrue(body["request_id"])

    def test_landing_epoch_change_does_not_use_airborne_health(self):
        client = self.client(); client.session_id = "s"
        def rpc(method,path,payload=None,**kw):
            if path == "/land":
                client.health.return_value["health"].update(localization_epoch="new",control_ready=False)
                return dict(ok=True,task_id="land")
            return dict(ok=True,task_id="land",status="arrived",stopped=True,
                        localization_error="reset during landing")
        client.rpc = Mock(side_effect=rpc)
        client.land()
        client.health.assert_not_called()
        self.assertIsNone(client._frame_identity)

    def test_takeoff_opt_in_and_fresh_session_init(self):
        client = self.client(auto_arm=True); client.depth_service = Mock()
        caps = dict(ok=True,backend="owl_ego",protocol_version=1,async_navigation=True,
                    cancel_and_hold=True,synchronized_observation=True,control_lease=True,
                    relative_xyz_yaw=True,software_takeoff=True)
        client.observe = Mock(return_value=observation())
        client.get_motion_tolerances = Mock(return_value={})
        client._init = Mock(return_value=dict(ok=True))
        def rpc(method,path,payload=None,**kw):
            if path == "/v21/capabilities": return caps
            if path == "/v21/session": return dict(ok=True,session_id="s")
            if path == "/takeoff":
                self.assertEqual(payload,{"auto_arm":True})
                return dict(ok=True)
            return dict(ok=True)
        client.rpc = Mock(side_effect=rpc)
        client.wait_stopped = Mock()
        client._health_state = Mock(side_effect=[dict(initialized=True,airborne=False),
                                                dict(initialized=True,airborne=True)])
        try:
            client.start()
            client._init.assert_called_once()
            self.assertTrue(any(c.args[1]=="/takeoff" for c in client.rpc.call_args_list))
        finally: client.close()
        caps["software_takeoff"] = False
        with self.assertRaisesRegex(RuntimeError,"software_takeoff"): client.start()
        self.assertIsNone(client.session_id)


class LeaseRecoveryTests(unittest.TestCase):
    def client(self):
        client=OwlEgoClient()
        client.session_id="original-session"
        client._lease_sent=100.
        client.event=Mock()
        return client

    def test_single_timeout_then_recovery_uses_send_time(self):
        client=self.client();now=[100.5]
        def rpc(*args,**kwargs):
            self.assertEqual(args[2]["session_id"],"original-session")
            if now[0]<102:
                now[0]+=2
                raise TimeoutError("temporary network timeout")
            now[0]+=.4
            return dict(ok=True)
        client.rpc=Mock(side_effect=rpc)
        with patch("robot_client.owl_ego.time.monotonic",side_effect=lambda:now[0]):
            self.assertTrue(client._renew_lease())
            self.assertEqual(client._lease_sent,100.)
            self.assertTrue(client._lease_recovering)
            now[0]=102.6
            self.assertTrue(client._renew_lease())
            self.assertEqual(client._lease_sent,102.6)
            self.assertFalse(client._lease_recovering)
            self.assertEqual(client._recovery_version,1)
            client.check_lease()

    def test_repeated_timeouts_use_original_deadline(self):
        client=self.client();now=[100.5];budgets=[]
        def fail(*args,timeout_s,**kwargs):
            budgets.append(timeout_s);now[0]+=timeout_s
            raise TimeoutError("no response")
        client.rpc=Mock(side_effect=fail)
        with patch("robot_client.owl_ego.time.monotonic",side_effect=lambda:now[0]):
            self.assertTrue(client._renew_lease())
            self.assertTrue(client._renew_lease())
            self.assertFalse(client._renew_lease())
            self.assertEqual(budgets,[2.,2.,.5])
            self.assertEqual(client._lease_sent,100.)
            with self.assertRaisesRegex(RuntimeError,"lease"):client.check_lease()
            self.assertFalse(client._renew_lease())
            self.assertEqual(client.rpc.call_count,3)

    def test_late_success_does_not_revive_session(self):
        client=self.client();now=[104.8]
        def late(*args,**kwargs):
            self.assertAlmostEqual(kwargs["timeout_s"],.2)
            now[0]=105.1;return dict(ok=True)
        client.rpc=Mock(side_effect=late)
        with patch("robot_client.owl_ego.time.monotonic",side_effect=lambda:now[0]):
            self.assertFalse(client._renew_lease())
            self.assertEqual(client._lease_sent,100.)
            self.assertIsNotNone(client._lease_error)

    def test_rejection_and_program_error_are_not_retried(self):
        for error in (RobotHTTPError(409,"/v21/heartbeat",'expired or operator takeover'),
                      ValueError("invalid JSON"),RuntimeError("program bug")):
            client=self.client();client.rpc=Mock(side_effect=error)
            with patch("robot_client.owl_ego.time.monotonic",return_value=101.):
                self.assertFalse(client._renew_lease())
                self.assertFalse(client._renew_lease())
            client.rpc.assert_called_once()
            self.assertTrue(client._lease_error)

    def test_transport_classification_preserves_wrapped_timeout(self):
        import urllib.error
        wrapped=RuntimeError("cannot connect")
        wrapped.__cause__=urllib.error.URLError(TimeoutError("timeout"))
        self.assertTrue(OwlEgoClient._transport_failure(wrapped))
        self.assertFalse(OwlEgoClient._transport_failure(RobotHTTPError(503,"/health","busy")))

    def test_new_navigation_waits_and_reconciles_original_task(self):
        client=self.client();client._lease_sent=time.monotonic()
        client._frame_identity=("odom","epoch-a")
        client.health=Mock(return_value=FakeClient().health())
        client._last_task_id="old-task"
        client.navigation_status=Mock(return_value=dict(status="arrived",stopped=True,task_id="old-task"))
        client.rpc=Mock(return_value=dict(ok=True,task_id="new-task"))
        client._begin_recovery()
        result=[]
        thread=threading.Thread(target=lambda:result.append(client.navigate(dict(x=20,y=0,z=100,yaw=0))))
        thread.start()
        try:
            time.sleep(.07)
            client.rpc.assert_not_called()
            client._renew_lease()
            thread.join(2)
            self.assertFalse(thread.is_alive())
            self.assertEqual(result,["new-task"])
            client.navigation_status.assert_called_once_with("old-task")
            self.assertEqual([c.args[1] for c in client.rpc.call_args_list],
                             ["/v21/heartbeat","/v21/navigation"])
        finally:
            client._fail_lease("test finished");thread.join(1)

    def test_recovered_failed_task_stays_failed_no_motion_replay(self):
        client=self.client();client._lease_sent=time.monotonic()
        client._frame_identity=("odom","epoch-a")
        client.health=Mock(return_value=FakeClient().health())
        client._recovery_version=1;client._last_task_id="failed-task"
        client.navigation_status=Mock(return_value=dict(status="failed",error="cancelled by watchdog"))
        client.rpc=Mock()
        with self.assertRaisesRegex(RuntimeError,"Original task failed"):
            client.navigate(dict(x=20,y=0,z=100,yaw=0))
        client.navigation_status.return_value=dict(status="arrived",stopped=True)
        with self.assertRaisesRegex(RuntimeError,"latched"):
            client.navigate(dict(x=20,y=0,z=100,yaw=0))
        client.rpc.assert_not_called()

    def test_health_timeout_recovers_read_once_without_replaying_motion(self):
        client=self.client();client._lease_sent=time.monotonic()
        client._frame_identity=("odom","epoch-a")
        client._last_task_id="original-task"
        client._lease_thread=threading.Thread(target=client._heartbeat_loop,daemon=True)
        health_calls=[0];paths=[]
        def wire(method,path,payload=None,**kwargs):
            paths.append(path)
            if path=="/health":
                health_calls[0]+=1
                if health_calls[0]==1:raise TimeoutError("health timed out")
                return FakeClient().health()
            if path.startswith("/v21/navigation/status"):
                return dict(ok=True,task_id="original-task",status="arrived",stopped=True)
            return dict(ok=True)
        with patch.object(BaseClient,"_request_json",side_effect=wire):
            client._lease_thread.start()
            try:
                client.flight_health()
                client.flight_health()  # Reconcile recovery observed during the first read.
                self.assertIn("/v21/heartbeat",paths)
                self.assertTrue(any(p.startswith("/v21/navigation/status") for p in paths))
                self.assertNotIn("/v21/navigation",paths)
                self.assertNotIn("/move_relative_xyz_yaw",paths)
            finally:client.close()

    def test_operator_takeover_latches_before_authority_error(self):
        client=self.client();client._lease_sent=time.monotonic()
        client._frame_identity=("odom","epoch-a")
        h=FakeClient().health();h["health"].update(control_ready=False,control_owner="operator")
        client.health=Mock(return_value=h)
        with self.assertRaisesRegex(RuntimeError,"operator takeover"):client.flight_health()
        client.rpc=Mock()
        with self.assertRaises(RuntimeError):client.land()
        client.rpc.assert_not_called()

    def test_blocking_landing_keeps_heartbeat_and_recovers(self):
        beats=[]
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*args):pass
            def do_GET(self):self.respond(dict(ok=True,task_id="land",status="arrived",stopped=True))
            def do_POST(self):
                self.rfile.read(int(self.headers.get("Content-Length",0)))
                if self.path=="/land":
                    time.sleep(1.8)
                    self.respond(dict(ok=True,task_id="land"))
                else:
                    if self.path=="/v21/heartbeat":beats.append(time.monotonic())
                    self.respond(dict(ok=True))
            def respond(self,data):
                raw=json.dumps(data).encode();self.send_response(200)
                self.send_header("Content-Length",str(len(raw)));self.end_headers();self.wfile.write(raw)
        server=ThreadingHTTPServer(("127.0.0.1",0),Handler)
        serving=threading.Thread(target=server.serve_forever,daemon=True);serving.start()
        client=OwlEgoClient(port=server.server_port);client.session_id="same"
        client._lease_sent=time.monotonic()
        original=client.rpc;failed=[False]
        def rpc(method,path,*args,**kw):
            if path=="/v21/heartbeat" and not failed[0]:
                failed[0]=True;raise TimeoutError("one transient timeout during land")
            return original(method,path,*args,**kw)
        client.rpc=rpc
        client._lease_thread=threading.Thread(target=client._heartbeat_loop,daemon=True)
        client._lease_thread.start()
        try:
            client.land()
            self.assertGreaterEqual(len(beats),2)
            self.assertEqual(client._recovery_version,1)
            self.assertIsNone(client._lease_error)
        finally:
            client.close();server.shutdown();server.server_close();serving.join(2)


class SimulatedHardware:
    """Simple timed pose interpolation; no ROS, EGO or FCU dynamics."""
    def __init__(self):
        self.lock = threading.RLock()
        self.session = None
        self.pose = np.array([0.,0.,1.,0.])  # Robot internal metres/radians
        self.tasks, self.commands = {}, []
        self.active = None
        self.epoch = "epoch-a"
        self.initialized = False
        self.airborne = True
        self.frame = 0
        self.frame_at = 0
        self.cached = None
        self.stop_until = 0

    def start(self): pass
    def close(self): pass

    def snapshot(self):
        with self.lock:
            now = time.monotonic()
            if self.active:
                t = self.tasks[self.active]
                fraction = min(1., (now-t["started"])/t["duration"])
                if t["status"] != "stopping":
                    self.pose = t["start"] + fraction*(t["goal"]-t["start"])
                    t["status"] = "planning" if fraction < .12 else "executing"
                if fraction >= 1:
                    t.update(status="cancelled" if t["status"]=="stopping" else "arrived",stopped=True)
                    self.active = None
                    self.stop_until = now + .025  # Historical stop is insufficient.
            h = dict(initialized=self.initialized,airborne=self.airborne,control_ready=True,
                     hold_ready=True,odom_ok=True,localization_epoch=self.epoch,rgb_ok=True,
                     active_task_id=self.active,stopped=self.active is None and now>=self.stop_until,
                     planner_state="not_required",planner_ok=False)
            if self.active:
                phase = self.tasks[self.active]["status"]
                h.update(planner_state="starting" if phase=="planning" else
                         "not_required" if phase=="stopping" else "ready",
                         planner_ok=phase=="executing")
            tasks = {key:{k:v for k,v in t.items() if k in ("task_id","status","stopped")}
                     for key,t in self.tasks.items()}
            return dict(session_id=self.session,pose=self.pose.tolist(),health=h,tasks=tasks)

    def command(self, op, data):
        with self.lock:
            snap = self.snapshot()
            self.commands.append((op,copy.deepcopy(data),time.monotonic(),self.pose.copy()))
            if op=="acquire": self.session=data["session_id"]
            elif op=="init": self.initialized=True
            elif op=="release": self.session=None
            elif op=="land":
                self.airborne=False; self.initialized=False; self.epoch="land-reset"
                self.active=None
                self.tasks[data["task_id"]]=dict(task_id=data["task_id"],status="arrived",stopped=True)
            elif op=="cancel":
                t=self.tasks[data["task_id"]]
                if t["status"] not in ("arrived","cancelled"):
                    t.update(status="stopping",started=time.monotonic(),duration=.08)
            elif op in ("navigate","relative"):
                if not snap["health"]["stopped"]:
                    return dict(ok=False,error="previous motion not confirmed stopped")
                goal=np.array(data.get("goal",self.pose),float)
                if op=="relative":
                    x,y,z,yaw=data["relative"]; a=self.pose[3]
                    goal=self.pose+np.array([math.cos(a)*x-math.sin(a)*y,
                                             math.sin(a)*x+math.cos(a)*y,z,yaw])
                self.active=data["task_id"]
                self.tasks[self.active]=dict(task_id=self.active,status="planning",stopped=False,
                    start=self.pose.copy(),goal=goal,started=time.monotonic(),duration=.65)
            return dict(ok=True,**({"task_id":data["task_id"]} if "task_id" in data else {}))

    def observation(self):
        from robot.controllers.owl_ego import public_pose
        with self.lock:
            self.snapshot()
            now=time.monotonic()
            if self.cached is None or now-self.frame_at>.025:
                self.frame+=1; self.frame_at=now
                data=approximate_wire(); data["frame_id"]=str(self.frame)
                data["pose"]=public_pose(self.pose); data["localization_epoch"]=self.epoch
                a=self.pose[3]
                r=np.array([[math.cos(a),-math.sin(a),0], [math.sin(a),math.cos(a),0],[0,0,1]])
                t=np.eye(4);t[:3,:3]=r@np.array([[0,0,1],[-1,0,0],[0,-1,0]])
                t[:3,3]=self.pose[:3]*100
                data["world_from_camera_optical_cm"]=t.tolist()
                self.cached=data
            return copy.deepcopy(self.cached)


class LocalRobotIntegrationTests(unittest.TestCase):
    def test_three_rounds_real_agent_track_client_and_robot_http(self):
        from robot.controllers.owl_ego import OwlEgoController
        from robot.server import run_http_server, NullKeepalive
        hw=SimulatedHardware()
        robot_config=yaml.safe_load((ROOT/"src/robot/config/owl_ego.yaml").read_text())
        controller=OwlEgoController(hardware=hw,config=robot_config)
        absent=[2]
        def synced_observation():
            from robot.controllers.owl_ego_observation import ObservationUnavailable
            if absent[0]:
                absent[0]-=1
                raise ObservationUnavailable("synthetic image awaiting odometry")
            return hw.observation()
        controller.observation=synced_observation  # Synthetic exposure, real HTTP/controller.
        server=run_http_server(controller,NullKeepalive(),"127.0.0.1",0)
        client=OwlEgoClient(port=server.server_port,allow_approximate_geometry=True)
        client.depth_service=Mock()
        client.depth_service.estimate_depth_cm.return_value=np.full((20,20),200.)
        class Tracker:
            frame_idx=0
            last_timing={}
            def reset(self): self.frame_idx=0
            def set_vis_dir(self,*args): pass
            def set_img_size(self,*args): pass
            def track_with_mask(self,frame,box=None):
                self.frame_idx+=1
                # Init, one forward adjustment, then centred at the desired size.
                b=np.array([8,8,12,12] if self.frame_idx<=2 else [6,6,14,14])
                mask=np.zeros((20,20),bool);mask[b[1]:b[3],b[0]:b[2]]=True
                return b,mask
        with tempfile.TemporaryDirectory() as directory:
            cfg=yaml.safe_load((ROOT/"src/agent/config/owl/v21.yaml").read_text())
            cfg["patrol"].update(capture_fps=40.,poll_interval_s=.02,min_depth_pixels=1)
            cfg["mission"]["waypoints"]=[dict(name=f"B{i}",x_cm=250.*(i+1),y_cm=0.,z_cm=0.,yaw_deg=0.) for i in range(3)]
            agent=PatrolAgent(config=cfg,client=client,detector=Mock(),tracker=Tracker(),
                              detector_name="sam3",tracker_name="sam2",vis_dir=str(Path(directory)/"vis"),save_vis=False)
            agent._log=lambda *args:None
            agent.save_depth=False; agent.action_sleep_s=0; agent.max_fb_step_cm=60
            records=[]
            original_event=agent.event
            def event(kind,**data):
                records.append((kind,copy.deepcopy(data)));original_event(kind,**data)
            agent.event=event;client.event=event
            triggered=set()
            def infer(obs):
                segment=agent.pipeline.segment if agent.pipeline else None
                if agent.phase=="PATROL":
                    if segment in triggered: return []
                    # Wait for a moving exposure, then model delayed inference.
                    if not hw.snapshot()["health"]["active_task_id"]: return []
                    triggered.add(segment);time.sleep(.2)
                box=[8,8,12,12]
                return [dict(box=box,confidence=.9,position_cm=agent.position(
                    obs,np.full((20,20),200.),box).tolist())]
            agent.infer_observation=infer
            try:
                agent.connect()
                result=agent.run_mission("synthetic bottle")
                agent.land()
                self.assertEqual(len(result),3)
                self.assertTrue(all(r["status"]=="completed" for r in result))
                phases=[data["phase"] for kind,data in records if kind=="phase"]
                self.assertEqual(phases.count("TRACK"),3)
                self.assertEqual(phases.count("RETURN_TO_ROUTE"),3)
                self.assertEqual(len([c for c in hw.commands if c[0]=="relative"]),3)
                claimed=[data for kind,data in records if kind=="target_claimed"]
                cancels=[c for c in hw.commands if c[0]=="cancel"]
                self.assertEqual(len(cancels),3)
                for hit,cancel in zip(claimed,cancels):
                    self.assertGreater(abs(cancel[3][0]*100-hit["capture_pose"]["x"]),12)
                goals=[data["target"] for kind,data in records if kind=="navigation_started"]
                for i,hit in enumerate(claimed):
                    self.assertEqual(goals[4*i+1:4*i+3],[hit["capture_pose"]]*2)
                    self.assertEqual(goals[4*i],goals[4*i+3])
                self.assertTrue(any(c[0]=="heartbeat" for c in hw.commands))
                self.assertIsNone(client._frame_identity)
                with agent.motion_csv_path.open(encoding="utf-8-sig",newline="") as stream:
                    motions=list(csv.DictReader(stream))
                self.assertEqual(list(motions[0]),["started_at","finished_at","phase","action",
                                                   "action_xyz_yaw","before","after","error"])
                relative=[r for r in motions if r["action"]=="xyz_yaw_hybrid"]
                self.assertEqual(len(relative),3)
                for row in relative:
                    before_x=float(row["before"].strip("()").split(",")[0])
                    after_x=float(row["after"].strip("()").split(",")[0])
                    self.assertAlmostEqual(after_x-before_x,30.,places=2)
                    self.assertEqual(row["action_xyz_yaw"],"(30.00, 0.00, 0.00, 0.00)")
                    self.assertEqual(row["error"].split(",")[2].strip(),"")
                self.assertEqual(motions[-1]["action"],"land")
                output=os.environ.get("V21_TEST_OUTPUT")
                if output:
                    path=Path(output);path.mkdir(parents=True,exist_ok=True)
                    (path/"integration_events.jsonl").write_text("".join(
                        json.dumps(dict(event=kind,**data),ensure_ascii=False)+"\n"
                        for kind,data in records),encoding="utf-8")
                    (path/"integration_result.json").write_text(json.dumps(dict(
                        scope="Real Agent/TRACK/Client/Robot HTTP; synthetic vision and interpolated hardware; no ROS/EGO/FCU",
                        rounds=3, targets=result, goals=goals,
                        cancelled_task_ids=[c[1]["task_id"] for c in cancels],
                        exposure_to_cancel_distance_cm=[abs(c[3][0]*100-h["capture_pose"]["x"])
                                                        for h,c in zip(claimed,cancels)]),indent=2),encoding="utf-8")
            finally:
                client.close();server.shutdown();server.server_close()


if __name__=="__main__": unittest.main()
