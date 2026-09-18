"""Offline geometry and actual bridge command-path tests; no ROS/FCU connection."""
import ast
import copy
import json
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock
import numpy as np
import yaml
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from robot.sensor_geometry import CameraPreflight, sensor_geometry
from robot.controllers.owl_ego_observation import build_observation
from robot.server import ApiHandler
CONFIG = yaml.safe_load((ROOT/'src/robot/config/owl_ego.yaml').read_text())


def header(stamp=10., frame='camera_link'):
    return S(stamp=S(to_sec=lambda:stamp,to_nsec=lambda:int(stamp*1e9)),frame_id=frame)


def gimbal(pitch=20., stamp=10.):
    a = math.radians(pitch)/2
    return S(header=header(stamp,'gimbal'), orientation=S(x=0,y=math.sin(a),z=0,w=math.cos(a)),
             orientation_covariance=[0]*9)


def image(stamp=10.):
    return S(header=header(stamp),width=1280,height=720,data=b'x')


class GeometryTest(unittest.TestCase):
    def test_composition_direction_and_camera_position(self):
        g = sensor_geometry(CONFIG)
        il = np.array(g['imu_from_lidar']); cl = np.array(g['camera_optical_from_lidar'])
        bc = np.array(g['body_from_camera_optical'])
        np.testing.assert_allclose(bc@cl,il,atol=1e-12)
        np.testing.assert_allclose(bc[:3,3],[.03649943,.02019536,-.09219387],atol=1e-6)
        self.assertEqual(g['quality'],'approximate')
        with self.assertRaises(ValueError):sensor_geometry({})
        c = copy.deepcopy(CONFIG);c['sensor_geometry']['imu_from_lidar'][0][0]=2
        with self.assertRaises(ValueError):sensor_geometry(c)

    def test_real_profile_rectification_resize_and_rotated_lever_arm(self):
        q=[0,0,math.sin(.3),math.cos(.3)]
        hist=[dict(stamp=t,frame='world',body='base_link',xyz=[1,2,3],q=q) for t in [9.98,10.02]]
        hw=S(c=copy.deepcopy(CONFIG),camera_snapshot=lambda:(image(),None,hist,'epoch',{}),
             now_s=lambda:10.05,current_epoch=lambda:'epoch',
             rgb_array=lambda m:np.zeros((720,1280,3),dtype=np.uint8))
        result=build_observation(hw)
        self.assertTrue(result['rectified']);self.assertEqual(result['calibration_quality'],'approximate')
        np.testing.assert_allclose(result['intrinsics'],np.diag([.5,.5,1])@np.array(CONFIG['hardware']['calibration']['K']))
        wb=np.eye(4);a=.6;wb[:3,:3]=[[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1]];wb[:3,3]=[1,2,3]
        expected=wb@np.array(sensor_geometry(CONFIG)['body_from_camera_optical']);expected[:3,3]*=100
        np.testing.assert_allclose(result['world_from_camera_optical_cm'],expected)
        hw.c['hardware']['calibration']['image_size']=[640,480]
        with self.assertRaises(ValueError):build_observation(hw)

    def test_common_http_interface_missing_and_configured(self):
        handler=object.__new__(ApiHandler)
        handler.keepalive=object();handler.command='GET';handler.path='/sensor_geometry'
        handler._json_response=Mock()
        handler.controller=S()
        handler._handle()
        self.assertEqual(handler._json_response.call_args.args[0],503)
        handler.controller=S(sensor_geometry_config=CONFIG)
        handler._handle()
        self.assertEqual(handler._json_response.call_args.args[0],200)
        self.assertEqual(handler._json_response.call_args.args[1]['sensor_geometry']['units'],'m')


class GateTest(unittest.TestCase):
    def setUp(self):
        self.gate=CameraPreflight(copy.deepcopy(CONFIG))

    def feed(self,pitch=20.,stamp=10.):
        self.gate.on_image(image(stamp),10.)
        self.gate.on_gimbal(gimbal(pitch,stamp),10.)

    def test_missing_good_boundary_outside_and_changed_after_ready(self):
        with self.assertRaises(ValueError):self.gate.require_ready(10.,10.)
        for pitch in [18.,19.,20.,21.]:
            self.feed(pitch);self.gate.require_ready(10.1,10.1)
        for pitch in [17.9,21.1,-20.]:
            self.feed(pitch)
            with self.assertRaises(ValueError):self.gate.require_ready(10.1,10.1)

    def test_stale_future_invalid_quaternion_and_resolution(self):
        for now,ros_now,stamp in [(10.6,10.,10.),(10.,10.6,10.),(10.,10.,11.),(10.,10.,0.)]:
            self.feed(stamp=stamp)
            with self.assertRaises(ValueError):self.gate.require_ready(now,ros_now)
        self.feed();m=gimbal();m.orientation.y=float('nan');self.gate.on_gimbal(m,10.)
        with self.assertRaises(ValueError):self.gate.require_ready(10.,10.)
        self.feed();m=gimbal();m.orientation_covariance[0]=-1;self.gate.on_gimbal(m,10.)
        with self.assertRaises(ValueError):self.gate.require_ready(10.,10.)
        self.feed();m=image();m.width=640;self.gate.on_image(m,10.)
        with self.assertRaises(ValueError):self.gate.require_ready(10.,10.)

    def test_other_drone_does_not_require_owl_pitch(self):
        c=copy.deepcopy(CONFIG);c.pop('camera_preflight')
        gate=CameraPreflight(c);gate.on_image(image(),10.);gate.require_ready(10.,10.)
        c.pop('sensor_geometry')
        with self.assertRaises(ValueError):CameraPreflight(c)

    def bridge_methods(self):
        tree=ast.parse((ROOT/'ros/owl_nav/scripts/owl_nav_node.py').read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Node')
        methods=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in ('command','software_takeoff')]
        ns=dict(json=json,math=math,time=S(monotonic=lambda:10.),
                rospy=S(Time=S(now=lambda:S(to_sec=lambda:10.))),
                CommandResponse=lambda **kw:S(**kw),Rejected=ValueError)
        exec(compile(ast.Module(body=methods,type_ignores=[]),'<actual bridge methods>','exec'),ns)
        return ns

    def test_bridge_rejects_before_core_mutation_and_land_bypasses_gate(self):
        ns=self.bridge_methods()
        core=S(landing=False,command=Mock(return_value=dict(ok=True)))
        node=S(lock=threading.RLock(),camera_preflight=self.gate,core=core,
               authorized=lambda now:True,publish_status=lambda now:{})
        for op in ['init','takeoff']:
            response=ns['command'](node,S(json=json.dumps(dict(op=op,deadline=11.))))
            self.assertFalse(json.loads(response.json)['ok']);core.command.assert_not_called()
        response=ns['command'](node,S(json=json.dumps(dict(op='land',deadline=11.))))
        self.assertTrue(json.loads(response.json)['ok']);core.command.assert_called_once()

    def test_software_takeoff_guard_rechecks_before_mode_and_arm(self):
        ns=self.bridge_methods()
        core=S(session='s',epoch='e',watchdog=Mock(),tasks={'t':dict(status='planning',auto_start_pending=True)},
               active='t',manual=False,enabled=True,fresh=lambda now:True,mode='OFFBOARD',armed=False)
        node=S(lock=threading.RLock(),core=core,camera_preflight=self.gate,stop=threading.Event(),
               authorized=lambda now:True,fcu_uncertain=False,bounded_fcu=Mock(),landed=1)
        ns.update(ExtendedState=S(LANDED_STATE_ON_GROUND=1),SetMode=object(),CommandBool=object())
        ns['rospy'].wait_for_service=lambda *a,**k:None
        for stage in ['mode','arm']:
            self.feed()
            def prepare(guard,mode,arm,warm):
                guard()  # fresh at acceptance / warm-up
                self.gate.on_gimbal(gimbal(30.),10.)
                (mode if stage=='mode' else arm)()
            ns['prepare']=prepare
            with self.assertRaisesRegex(ValueError,'camera preflight'):ns['software_takeoff'](node,'t')
            node.bounded_fcu.assert_not_called()


if __name__=='__main__':unittest.main()
