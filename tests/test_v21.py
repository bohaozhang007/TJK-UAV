"""Offline tests: no model weights, ROS, drone connection or flight commands."""
import base64
import copy
from dataclasses import replace
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"src"))
import cv2
import numpy as np
import yaml

from agent.tjk.patrol import PerceptionPipeline, TargetGeometryError, TargetMemory, target_position
from agent.tjk.v21 import PatrolAgent, validate_patrol_config
from agent.tjk.v20 import FlightSafetyError
from robot_client.owl_ego import Observation, OwlEgoClient, decode_observation


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
                rectified=True, rgb_jpeg_base64=base64.b64encode(jpg).decode(),
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
                    result=dict(health=dict(initialized=True,airborne=True))
                elif self.path=="/motion_tolerances":
                    result=dict(motion_tolerances=dict(position_tolerance_cm=15,yaw_tolerance_deg=5,
                                position_error_metric="euclidean_3d",source="owl_ego"))
                elif self.path=="/v21/navigation": result=dict(task_id="nav-1")
                elif self.path.startswith("/v21/navigation/status"):
                    result=dict(task_id="nav-1",status="cancelled",stopped=True)
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


class FakeClient:
    def __init__(self):
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
                    odom_ok=True,rgb_ok=True,planner_ok=True,localization_epoch="epoch-a"))
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
        config["patrol"].update(capture_fps=60.,poll_interval_s=.02)
        self.config=config
        self.client=FakeClient()
        detector=Mock()
        detector.cfg.confidence_threshold=.5
        self.agent=PatrolAgent(config=config,client=self.client,detector=detector,
                               tracker=Mock(),detector_name="sam3",tracker_name="sam2",
                               vis_dir=self.temp.name,save_vis=False)
        self.agent.event=Mock()
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


if __name__=="__main__": unittest.main()
