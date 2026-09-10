"""Offline contract/safety regression tests; never connects to ROS or FCU."""
import copy
import io
import json
import math
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import urllib.request
import urllib.error
import uuid
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'ros/owl_nav/src')]
from owl_nav.core import FlightCore,Polynomial,Rejected
from owl_nav.planner import PlannerProcess
from robot.controllers.owl_ego import OwlEgoController,ApiError,enu_pose,public_pose
from robot.hardware.owl_ego import OwlEgoHardware
from robot.controllers.owl_ego_observation import interpolate_pose,rotation,transform_sync_error,build_observation,camera_intrinsics,fixed_optical_rotation
from robot.server import run_http_server,NullKeepalive
CONFIG=yaml.safe_load((ROOT/'src/robot/config/owl_ego.yaml').read_text())


class FramesTest(unittest.TestCase):
    def test_vendor_transforms_all_setpoint_vectors(self):
        from owl_nav.frames import Frames
        f=Frames('owl_vendor_world')
        for yaw in (0.,.7,-2.,math.pi):
            p,v,a,y=f.setpoint([1,2,3],[4,5,6],[7,8,9],yaw)
            np.testing.assert_allclose(p,[-2,1,3])
            np.testing.assert_allclose(v,[-5,4,6])
            np.testing.assert_allclose(a,[-8,7,9])
            self.assertAlmostEqual(math.sin(y-yaw),1.)
            # Vendor output map->world leaves measured heading unchanged.
            self.assertAlmostEqual(math.sin(y-math.pi/2-yaw),0.)

    def test_tilted_yaw_rate_accounts_for_vendor_axis_swap(self):
        from owl_nav.frames import Frames
        r=.3;p=.2
        rotation=np.array([[math.cos(p),math.sin(p)*math.sin(r),math.sin(p)*math.cos(r)],
                           [0,math.cos(r),-math.sin(r)],[-math.sin(p),math.cos(p)*math.sin(r),math.cos(p)*math.cos(r)]])
        expected=(math.sin(r)*.4+math.cos(r)*.2)/math.cos(p)
        self.assertAlmostEqual(Frames('standard_enu').yaw_rate(rotation,[.1,.4,.2]),expected)
        self.assertAlmostEqual(Frames('owl_vendor_world').yaw_rate(rotation,[.4,-.1,.2]),expected)

    def test_vendor_twist_is_already_world(self):
        from owl_nav.frames import Frames
        rotation=np.array([[0.,-1,0],[1,0,0],[0,0,1]])
        np.testing.assert_allclose(Frames('owl_vendor_world').world_velocity(rotation,[1,2,3]),[1,2,3])
        np.testing.assert_allclose(Frames('standard_enu').world_velocity(rotation,[1,2,3]),[-2,1,3])

    def test_standard_transport_unchanged_and_invalid_rejected(self):
        from owl_nav.frames import Frames
        values=([1,2,3],[4,5,6],[7,8,9],.8)
        self.assertEqual(Frames('standard_enu').setpoint(*values),values)
        with self.assertRaises(ValueError):Frames('guess')


class AlignmentTest(unittest.TestCase):
    def test_matching_yaw_wrap_and_stale_reference(self):
        from owl_nav.frames import Alignment
        a=Alignment();yaw=3.
        a.add('map',1.,[-2,1,3],(yaw+math.pi/2+math.pi)%(2*math.pi)-math.pi)
        a.add('lio',1.,[1,2,3],yaw)
        a.check(1.,[1,2,3],yaw,10.)
        self.assertTrue(a.ready(10.))
        self.assertFalse(a.ready(10.6))

    def test_delayed_map_matches_its_own_odom_during_fast_yaw(self):
        from owl_nav.frames import Alignment
        a=Alignment()
        a.check(1.,[0,0,1],0.,10.)
        a.check(1.04,[0,0,1],.08,10.04)
        a.add('map',1.,[0,0,1],math.pi/2)
        a.add('lio',1.04,[0,0,1],.08)
        a.check(1.04,[0,0,1],.08,10.06)
        self.assertTrue(a.ready(10.06))
        # Old matched data cannot keep renewing freshness.
        a.check(1.04,[0,0,1],.08,10.6)
        self.assertFalse(a.ready(10.6))

    def test_map_arrives_before_odom_and_real_offset_is_rejected(self):
        from owl_nav.frames import Alignment
        a=Alignment();a.add('map',1.,[0,0,1],math.pi/2)
        a.check(1.,[0,0,1],0.,10.)
        a.add('map',1.04,[0,0,1],math.pi/2+.3)
        with self.assertRaises(ValueError):a.check(1.04,[0,0,1],0.,10.04)

    def test_wrong_axes_and_lio_origin_rejected(self):
        from owl_nav.frames import Alignment
        a=Alignment();a.add('map',1.,[1,2,3],0.)
        with self.assertRaises(ValueError):a.check(1.,[1,2,3],0.,10.)
        a=Alignment();a.add('lio',1.,[2,2,3],0.)
        with self.assertRaises(ValueError):a.check(1.,[1,2,3],0.,10.)

    def test_unsynchronized_samples_do_not_validate(self):
        from owl_nav.frames import Alignment
        a=Alignment();a.add('map',1.,[0,0,1],math.pi/2);a.add('lio',1.,[0,0,1],0.)
        a.check(1.1,[0,0,1],0.,10.)
        self.assertFalse(a.ready(10.))


class CloudTest(unittest.TestCase):
    def message(self,xyz):
        values=np.asarray(xyz,dtype='<f4')
        return SimpleNamespace(width=len(values),height=1,point_step=12,row_step=12*len(values),
            data=values.tobytes(),is_bigendian=False,
            fields=[SimpleNamespace(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate('xyz')])

    def test_valid_and_partially_nan_cloud(self):
        from owl_nav.cloud import validate
        self.assertEqual(validate(self.message([[1,2,3],[float('nan'),0,1]])),1)

    def test_invalid_cloud_rejected(self):
        from owl_nav.cloud import validate
        for xyz in ([],[[float('nan')]*3],[[float('inf'),0,0]]):
            with self.assertRaises(ValueError):validate(self.message(xyz))
        m=self.message([[1,2,3]]);m.fields[0].datatype=8
        with self.assertRaises(ValueError):validate(m)
        m=self.message([[1,2,3]]);m.data=b''
        with self.assertRaises(ValueError):validate(m)


class SoftwareTakeoffTest(unittest.TestCase):
    def run_prepare(self, failure=None):
        from owl_nav.software_takeoff import prepare
        state=dict(mode='POSCTL',armed=False)
        calls=[];now=[0.];valid=[True]
        def guard():
            if not valid[0]:raise RuntimeError('cancelled')
            return state.copy()
        def mode():
            calls.append('mode')
            if failure=='mode':raise RuntimeError('rejected')
            if failure!='timeout':state['mode']='OFFBOARD'
            if failure=='cancel':valid[0]=False
        def arm():
            calls.append('arm');state['armed']=True
            if failure=='late_arm':valid[0]=False
        def sleep(dt):now[0]+=dt
        try:
            prepare(guard,mode,arm,lambda:now[0]>=1.2,lambda:now[0],sleep)
        except RuntimeError:
            if failure is None:raise
        else:
            self.assertIsNone(failure)
        self.assertGreaterEqual(now[0],1.2)
        return calls

    def test_order_and_warmup(self):self.assertEqual(self.run_prepare(),['mode','arm'])
    def test_mode_rejected_no_arm(self):self.assertEqual(self.run_prepare('mode'),['mode'])
    def test_cancel_before_arm(self):self.assertEqual(self.run_prepare('cancel'),['mode'])
    def test_timeout_no_retry(self):self.assertEqual(self.run_prepare('timeout'),['mode'])
    def test_late_arm_cannot_complete_cancelled_task(self):self.assertEqual(self.run_prepare('late_arm'),['mode','arm'])


class CoreTest(unittest.TestCase):
    def setUp(self):
        self.c=FlightCore(CONFIG['control'])
        self.now=10.
        self.stable()
        self.c.command('acquire',{'session_id':'s'},self.now)
        self.c.command('init',{'session_id':'s','flight_authorized':True},self.now)
        self.c.enabled=True
        self.c.hold=self.c.pose.copy()

    def feed(self,p=(0,0,1),yaw=0,v=(0,0,0),dt=.1):
        self.now+=dt
        self.c.update_state(True,True,True,'OFFBOARD',self.now)
        self.c.odometry(p,yaw,v,0,self.now,'world',self.now)

    def cmd(self,op,**kw):
        return self.c.command(op,dict(session_id='s',**kw),self.now)

    def nav(self,tid='a',goal=(1,0,1,0)):
        return self.cmd('navigate',task_id=tid,goal=list(goal),localization_epoch=self.c.epoch)

    def stable(self,p=(0,0,1),yaw=0):
        for _ in range(8): self.feed(p,yaw)

    def test_current_stop_is_revoked_after_arrival_and_reports_reason(self):
        self.nav(goal=(0,0,1,0));self.stable()
        self.assertEqual(self.c.tasks['a']['status'],'arrived')
        self.feed(p=(.06,0,1),v=(.12,0,0))
        h=self.c.status(self.now)
        self.assertFalse(h['stopped'])
        self.assertTrue(self.c.tasks['a']['stopped'])
        self.assertEqual(h['stop_diagnostics']['speed_m_s'],.12)
        self.assertEqual(h['stop_diagnostics']['stable_samples'],0)
        with self.assertRaisesRegex(Rejected,'previous motion'):self.nav('b')
        self.stable();self.nav('b')

    def test_stop_diagnostics_before_odom_is_json_safe(self):
        h=FlightCore(CONFIG['control']).status(0)
        self.assertIsNone(h['stop_diagnostics']['speed_m_s'])
        json.dumps(h,allow_nan=False)

    def test_cancel_accepts_small_pose_jitter_despite_biased_vertical_twist(self):
        self.nav();self.cmd('cancel',task_id='a')
        for i in range(30):
            self.feed(p=(.006*math.sin(i),0,1+.008*math.cos(i)),v=(0,0,-.124),dt=.033)
        self.assertTrue(self.c.stopped)
        self.assertEqual(self.c.tasks['a']['status'],'cancelled')
        self.assertGreater(self.c.status(self.now)['stop_diagnostics']['speed_m_s'],.1)

    def test_level_relative_sequence_does_not_accumulate_altitude_offset(self):
        self.nav(goal=(0,0,1,0));self.stable((0,0,1.1))
        for i,relative in enumerate([[.3,0,0,0],[0,-.3,0,0],[-.3,0,0,0],[0,0,0,.35],[.2,.2,0,-.35]]):
            self.cmd('heartbeat')
            self.cmd('relative',task_id=str(i),relative=relative,localization_epoch=self.c.epoch)
            task=self.c.tasks[str(i)];goal=task['goal']
            self.assertAlmostEqual(goal[2],1.)
            self.assertAlmostEqual(self.c.hold[2],1.)
            # Inspect the actual hold/direct-yaw output before a planner exists.
            self.assertAlmostEqual(self.c.tick(self.now,self.now)[0][2],1.)
            if i==3:self.assertFalse(task['planner_required'])
            for _ in range(8):self.feed((goal[0],goal[1],goal[2]+.10),yaw=goal[3],v=(0,0,-.124))
            self.assertEqual(task['status'],'arrived')
            self.assertAlmostEqual(self.c.pose[2],1.1)
            self.assertAlmostEqual(self.c.hold[2],1.)

    def test_explicit_vertical_relative_uses_measured_altitude_then_retains_goal(self):
        self.stable((0,0,1.1))
        self.cmd('relative',task_id='up',relative=[0,0,.2,0],localization_epoch=self.c.epoch)
        self.assertAlmostEqual(self.c.tasks['up']['goal'][2],1.3)
        self.stable((0,0,1.4))
        self.cmd('relative',task_id='level',relative=[.3,0,0,0],localization_epoch=self.c.epoch)
        self.assertAlmostEqual(self.c.tasks['level']['goal'][2],1.3)
        self.cmd('cancel',task_id='level');self.stable((0,0,1.4))
        self.assertAlmostEqual(self.c.hold[2],1.4)
        self.cmd('relative',task_id='after_cancel',relative=[.3,0,0,0],localization_epoch=self.c.epoch)
        self.assertAlmostEqual(self.c.tasks['after_cancel']['goal'][2],1.4)

    def test_absolute_navigation_keeps_previous_Z_while_planning_and_recaptures_on_cancel(self):
        self.nav(goal=(0,0,1,0));self.stable((0,0,1.1))
        self.cmd('navigate',task_id='side',goal=[0,1,1,0],localization_epoch=self.c.epoch)
        self.assertAlmostEqual(self.c.tick(self.now,self.now)[0][2],1.)
        self.assertAlmostEqual(self.c.hold[2],1.)
        self.cmd('cancel',task_id='side');self.stable((0,0,1.1))
        self.assertAlmostEqual(self.c.hold[2],1.1)
        self.cmd('navigate',task_id='up',goal=[0,1,1.5,0],localization_epoch=self.c.epoch)
        self.assertAlmostEqual(self.c.tasks['up']['goal'][2],1.5)
        self.assertAlmostEqual(self.c.tick(self.now,self.now)[0][2],1.1)

    def test_invalid_relative_does_not_replace_altitude_reference(self):
        self.stable((0,0,1.1));before=self.c.hold.copy()
        with self.assertRaises(Rejected):
            self.cmd('relative',task_id='bad',relative=[.3,0,0,0],localization_epoch='old')
        np.testing.assert_array_equal(self.c.hold,before)
        self.c.reset('new origin')
        self.assertIsNone(self.c.hold)
        with self.assertRaises(Rejected):
            self.cmd('relative',task_id='reset',relative=[.3,0,0,0],localization_epoch=self.c.epoch)

    def test_level_relative_excessive_height_error_cannot_complete(self):
        self.cmd('relative',task_id='level',relative=[.3,0,0,0],localization_epoch=self.c.epoch)
        self.stable((.3,0,1.2))
        self.assertNotEqual(self.c.tasks['level']['status'],'arrived')
        with self.assertRaises(Rejected):
            self.cmd('relative',task_id='next',relative=[.3,0,0,0],localization_epoch=self.c.epoch)

    def test_position_window_rejects_drift_and_return_swing_even_with_zero_twist(self):
        self.nav();self.cmd('cancel',task_id='a')
        for i in range(30):self.feed(p=(i*.02,0,1),dt=.05)
        self.assertFalse(self.c.stopped)
        for i in range(30):self.feed(p=(.5+.08*math.sin(i*math.pi/3),0,1),dt=.05)
        self.assertFalse(self.c.stopped)
        self.assertEqual(self.c.tasks['a']['status'],'stopping')

    def test_pose_window_handles_yaw_wrap_and_reset_without_old_samples(self):
        self.stable(yaw=math.pi-.005)
        for i in range(12):self.feed(yaw=math.pi-.005 if i%2 else -math.pi+.005,dt=.05)
        self.assertTrue(self.c.stopped)
        self.c.reset('new origin')
        self.assertFalse(self.c.stopped)
        self.assertEqual(len(self.c.pose_window),0)
        for i in range(3):self.feed(dt=.05)
        self.assertFalse(self.c.stopped)

    def test_pose_yaw_motion_does_not_count_as_stopped_with_zero_gyro(self):
        self.nav();self.cmd('cancel',task_id='a')
        for i in range(20):self.feed(yaw=i*.03,dt=.05)
        self.assertFalse(self.c.stopped)

    def test_relative_axes_through_public_world_and_vendor_map(self):
        from owl_nav.frames import Frames
        for angle in (0.,math.pi/2,-math.pi/2,math.pi-.01):
            self.setUp();self.stable(yaw=angle)
            self.cmd('init',flight_authorized=True)
            origin=self.c.pose.copy()
            self.cmd('relative',task_id='axes',relative=[.4,-.3,.2,-.35],localization_epoch=self.c.epoch)
            goal=self.c.tasks['axes']['goal']
            map_origin=Frames('owl_vendor_world').setpoint(origin[:3],[0]*3,[0]*3,angle)
            map_goal=Frames('owl_vendor_world').setpoint(goal[:3],[0]*3,[0]*3,goal[3])
            d=map_goal[0]-map_origin[0];heading=map_origin[3]
            self.assertAlmostEqual(d[0]*math.cos(heading)+d[1]*math.sin(heading),.4)
            self.assertAlmostEqual(d[0]*math.sin(heading)-d[1]*math.cos(heading),.3)
            self.assertAlmostEqual(d[2],.2)
            self.assertAlmostEqual((map_goal[3]-heading+math.pi)%(2*math.pi)-math.pi,-.35)
            np.testing.assert_allclose(enu_pose(public_pose(goal),30),goal,atol=1e-12)

    def test_cancel_requires_fresh_stable_stop(self):
        self.nav(); self.cmd('cancel',task_id='a')
        self.assertEqual(self.c.tasks['a']['status'],'stopping')
        self.assertEqual(self.c.status(self.now)['planner_state'],'not_required')
        for i in range(1,9): self.feed((.03*i,0,1),v=(.3,0,0))
        self.assertEqual(self.c.tasks['a']['status'],'stopping')
        self.stable((.24,0,1))
        self.assertEqual(self.c.tasks['a']['status'],'cancelled')
        self.assertTrue(self.c.tasks['a']['stopped'])

    def test_duplicate_odom_does_not_confirm_stop(self):
        self.nav();self.cmd('cancel',task_id='a')
        for _ in range(50): self.c.odometry([0,0,1],0,[0,0,0],0,self.now,'world',self.now)
        self.assertEqual(self.c.tasks['a']['status'],'stopping')

    def test_arrival_yaw_speed_and_cancel_race(self):
        self.nav(goal=(1,0,1,1))
        for x in [.2,.4,.6,.8]: self.feed((x,0,1))
        self.stable((1,0,1),0)
        self.assertNotEqual(self.c.tasks['a']['status'],'arrived')
        self.feed((1,0,1),.5)
        self.stable((1,0,1),1)
        self.assertEqual(self.c.tasks['a']['status'],'arrived')
        self.cmd('cancel',task_id='a')
        self.assertEqual(self.c.tasks['a']['status'],'arrived')

    def test_cancel_old_terminal_does_not_touch_new(self):
        self.nav();self.cmd('cancel',task_id='a');self.stable();self.nav('b')
        gen=self.c.generation
        self.cmd('cancel',task_id='a')
        self.assertEqual(gen,self.c.generation)
        self.assertEqual(self.c.active,'b')

    def test_late_trajectory_excluded_after_resume(self):
        self.nav();old=self.c.generation
        self.cmd('cancel',task_id='a');self.stable();self.nav('b')
        traj=SimpleNamespace(id=1,start=self.now,duration=5)
        self.assertFalse(self.c.trajectory_received(old,traj,self.now,self.now))
        self.assertTrue(self.c.trajectory_received(self.c.generation,traj,self.now,self.now))
        self.assertFalse(self.c.trajectory_received(self.c.generation,traj,self.now,self.now))

    def test_foreign_unknown_busy_epoch_nonfinite(self):
        with self.assertRaises(Rejected): self.c.command('heartbeat',{'session_id':'x'},self.now)
        with self.assertRaises(Rejected): self.cmd('cancel',task_id='missing')
        with self.assertRaises(Rejected): self.cmd('navigate',task_id='a',goal=[0,0,1,0],localization_epoch='bad')
        with self.assertRaises(Rejected): self.nav(goal=(math.nan,0,1,0))
        self.nav()
        with self.assertRaises(Rejected): self.nav('b')

    def test_lease_expiry_holds_and_cannot_revive(self):
        self.nav();self.now+=5
        # Fresh telemetry lets the bridge hold despite Agent loss.
        self.feed()
        self.c.tick(self.now,self.now)
        self.assertIsNone(self.c.session)
        self.assertEqual(self.c.tasks['a']['status'],'failed')
        self.assertIsNone(self.c.generation)
        with self.assertRaises(Rejected):self.cmd('heartbeat')
        with self.assertRaises(Rejected):self.c.command('acquire',{'session_id':'s'},self.now)

    def test_localization_reset_aborts(self):
        self.nav();epoch=self.c.epoch
        self.feed((10,0,1))
        self.assertNotEqual(epoch,self.c.epoch)
        self.assertFalse(self.c.enabled)
        self.assertEqual(self.c.tasks['a']['status'],'failed')

    def test_manual_takeover_latches(self):
        self.nav()
        self.c.update_state(True,True,True,'POSCTL',self.now)
        self.c.update_state(True,True,True,'OFFBOARD',self.now)
        self.assertTrue(self.c.manual)
        self.assertIsNone(self.c.tick(self.now,self.now))
        with self.assertRaises(Rejected):self.cmd('land',task_id='land')

    def test_invalid_reference_diagnostics_remain_json_serializable(self):
        self.nav();self.c.planner_heartbeat(self.c.generation,self.now)
        t=SimpleNamespace(id=1,start=self.now,duration=5,
                          sample=lambda stamp:(np.array([float('inf'),0,1]),np.zeros(3),np.zeros(3)))
        self.c.trajectory_received(self.c.generation,t,self.now,self.now)
        self.c.tick(self.now,self.now)
        self.assertEqual(self.c.tasks['a']['status'],'failed')
        json.dumps(self.c.tasks,allow_nan=False)
        self.assertIsNone(self.c.tasks['a']['execution_error']['tracking_error_m'])

    def test_discontinuous_position_reference_fails_and_holds(self):
        self.nav();self.c.planner_heartbeat(self.c.generation,self.now)
        t=SimpleNamespace(id=1,start=self.now,duration=5,
                          sample=lambda stamp:(np.array([5.,0,1]),np.zeros(3),np.zeros(3)))
        self.c.trajectory_received(self.c.generation,t,self.now,self.now)
        output=self.c.tick(self.now,self.now)
        self.assertEqual(self.c.tasks['a']['status'],'failed')
        np.testing.assert_allclose(output[0],self.c.pose[:3])

    def test_land_allowed_outside_navigation_world_limit(self):
        self.c.pose[0]=self.c.c['world_limit_m']+1
        self.cmd('land',task_id='land')
        self.assertTrue(self.c.landing)

    def test_landing_keeps_hold_until_mode_confirmation(self):
        self.nav();self.cmd('land',task_id='land')
        self.assertTrue(self.c.landing)
        np.testing.assert_allclose(self.c.tick(self.now,self.now)[0],self.c.pose[:3])
        self.c.update_state(True,True,True,'AUTO.LAND',self.now)
        self.assertIsNone(self.c.tick(self.now,self.now))

    def test_localization_reset_preserves_pending_land_and_revokes_world_output(self):
        self.nav();self.cmd('land',task_id='land');epoch=self.c.epoch
        self.c.reset('lio/world mismatch')
        self.assertNotEqual(self.c.epoch,epoch)
        self.assertEqual(self.c.active,'land')
        self.assertTrue(self.c.landing)
        self.assertNotEqual(self.c.tasks['land']['status'],'failed')
        self.assertEqual(self.c.tasks['land']['localization_error'],'lio/world mismatch')
        self.assertIsNone(self.c.hold)
        self.assertIsNone(self.c.tick(self.now,self.now))
        self.c.update_state(True,True,True,'AUTO.LAND',self.now)
        self.c.reset('another localization reset')
        self.c.update_extended_state(0,self.now)
        self.c.update_state(True,False,False,'AUTO.LAND',self.now)
        self.assertNotEqual(self.c.tasks['land']['status'],'arrived')
        self.c.update_extended_state(1,self.now)
        self.assertEqual(self.c.tasks['land']['status'],'arrived')
        self.assertFalse(self.c.initialized)

    def test_reset_during_landing_does_not_bypass_pilot_takeover(self):
        self.cmd('land',task_id='land');self.c.reset('lio mismatch')
        self.c.update_state(True,True,True,'POSCTL',self.now)
        self.assertTrue(self.c.manual)
        self.assertEqual(self.c.tasks['land']['status'],'failed')

    def test_reset_during_landing_does_not_bypass_lease_expiry(self):
        self.cmd('land',task_id='land');self.c.reset('lio mismatch')
        self.c.watchdog(self.now+5.)
        self.assertEqual(self.c.tasks['land']['status'],'failed')
        self.assertIsNone(self.c.session)

    def test_pilot_takeover_before_land_ack_is_latched(self):
        self.nav();self.cmd('land',task_id='land')
        self.c.update_state(True,True,True,'POSCTL',self.now)
        self.assertTrue(self.c.manual)
        self.assertIsNone(self.c.tick(self.now,self.now))
        self.c.update_state(True,True,True,'AUTO.LAND',self.now)
        self.assertTrue(self.c.manual)

    def test_landed_session_cannot_restart_takeoff_without_init(self):
        self.cmd('land',task_id='land')
        self.c.update_extended_state(1,self.now)
        self.c.update_state(True,False,False,'AUTO.LAND',self.now)
        self.assertFalse(self.c.initialized)
        with self.assertRaises(Rejected):
            self.cmd('takeoff',task_id='late',localization_epoch=self.c.epoch)

    def test_landing_cannot_be_cancelled_by_navigation(self):
        self.nav();self.cmd('land',task_id='land')
        with self.assertRaises(Rejected):self.cmd('cancel',task_id='land')
        self.assertTrue(self.c.landing)

    def test_invalid_odometry_revokes_output(self):
        self.nav();old=self.c.epoch
        self.c.odometry([float('nan'),0,1],0,[0,0,0],0,self.now,'world',self.now)
        self.assertNotEqual(self.c.epoch,old)
        self.assertFalse(self.c.fresh(self.now))
        self.assertIsNone(self.c.tick(self.now,self.now))

    def test_cancel_yaw_holds_measured_heading(self):
        self.nav(goal=(0,0,1,1.))
        self.c.yaw=.8
        self.cmd('cancel',task_id='a')
        self.assertEqual(self.c.tick(self.now,self.now)[3],self.c.pose[3])

    def test_direct_yaw_rejects_unsolicited_trajectory(self):
        self.nav(goal=(0,0,1,1.))
        t=SimpleNamespace(id=1,start=self.now,duration=5)
        self.assertFalse(self.c.trajectory_received(self.c.generation,t,self.now,self.now))

    def test_land_preempts(self):
        self.nav();self.cmd('land',task_id='land')
        self.assertEqual(self.c.tasks['a']['status'],'failed')
        self.assertIsNone(self.c.generation)
        self.c.update_extended_state(1,self.now)
        self.c.update_state(True,False,False,'AUTO.LAND',self.now)
        self.assertEqual(self.c.tasks['land']['status'],'arrived')

    def test_independent_yaw_and_backward(self):
        self.nav(goal=(-1,0,1,0))
        self.c.planner_at=self.now
        out=self.c.tick(self.now,self.now)
        self.assertEqual(out[3],0)
        self.cmd('cancel',task_id='a');self.stable()
        self.nav('b',goal=(0,0,1,math.pi/2))
        previous=self.c.tick(self.now,self.now)[3]
        self.now+=.1;out=self.c.tick(self.now,self.now)
        self.assertLessEqual(out[3]-previous,CONFIG['control']['yaw_rate_rad_s']*.1+1e-8)
        np.testing.assert_allclose(out[0],[0,0,1])

    def test_robot_restart_epoch(self):
        self.nav();epoch=self.c.epoch
        self.c.command('robot_restart',{},self.now)
        self.assertNotEqual(epoch,self.c.epoch)
        self.assertIsNone(self.c.session)
        self.assertIsNone(self.c.generation)

    def test_expired_polynomial_does_not_hold_endpoint(self):
        self.nav()
        msg=SimpleNamespace(start_time=SimpleNamespace(to_sec=lambda:self.now-4),traj_id=1,order=5,
                            duration=[1.],coef_x=[0,0,0,0,0,9],coef_y=[0]*6,coef_z=[0,0,0,0,0,1])
        self.c.trajectory=Polynomial(msg)
        self.c.planner_at=self.now
        out=self.c.tick(self.now,self.now)
        self.assertEqual(self.c.tasks['a']['status'],'failed')
        np.testing.assert_allclose(out[0],[0,0,1])

    def short_position_large_yaw(self):
        self.nav(goal=(.1,0,1,math.pi/2))
        trajectory=SimpleNamespace(id=1,start=self.now,duration=1.,
            sample=lambda stamp:(np.array([.1,0,1]),np.array([.1,0,0]),np.array([.2,0,0])))
        self.assertTrue(self.c.trajectory_received(self.c.generation,trajectory,self.now,self.now))
        return self.now

    def test_short_position_path_waits_for_large_yaw_and_holds_endpoint(self):
        start=self.short_position_large_yaw()
        for i in range(1,46):
            self.feed((.1,0,1),yaw=min(math.pi/2,math.radians(24)*i*.1))
            self.cmd('heartbeat');self.c.planner_heartbeat(self.c.generation,self.now)
            out=self.c.tick(self.now,self.now)
            if 1.1<self.now-start<4.:
                np.testing.assert_allclose(out[0],[.1,0,1])
                np.testing.assert_array_equal(out[1],np.zeros(3))
                np.testing.assert_array_equal(out[2],np.zeros(3))
            if 3.2<self.now-start<3.4:self.assertEqual(self.c.tasks['a']['status'],'executing')
        self.assertEqual(self.c.tasks['a']['status'],'arrived')
        self.assertTrue(self.c.tasks['a']['stopped'])

    def test_yaw_allowance_is_bounded_when_heading_never_moves(self):
        self.short_position_large_yaw()
        for _ in range(58):
            self.feed((.1,0,1));self.cmd('heartbeat')
            self.c.planner_heartbeat(self.c.generation,self.now);self.c.tick(self.now,self.now)
        self.assertEqual(self.c.tasks['a']['status'],'failed')
        self.assertEqual(self.c.tasks['a']['error'],'trajectory expired before measured arrival')

    def test_cancellation_during_yaw_allowance_discards_endpoint(self):
        self.short_position_large_yaw()
        for _ in range(33):
            self.feed((.12,0,1),yaw=.7);self.cmd('heartbeat')
            self.c.planner_heartbeat(self.c.generation,self.now);self.c.tick(self.now,self.now)
        self.assertEqual(self.c.tasks['a']['status'],'executing')
        self.cmd('cancel',task_id='a')
        self.assertIsNone(self.c.trajectory)
        np.testing.assert_allclose(self.c.tick(self.now,self.now)[0],[.12,0,1])
        self.stable((.12,0,1),yaw=.7)
        self.assertEqual(self.c.tasks['a']['status'],'cancelled')

    def test_yaw_budget_wraps_shortest_angle_and_replans_do_not_renew_it(self):
        self.stable(yaw=math.radians(179));self.cmd('init',flight_authorized=True)
        self.nav(goal=(.1,0,1,math.radians(-179)))
        t=SimpleNamespace(id=1,start=self.now,duration=1.)
        self.assertTrue(self.c.trajectory_received(self.c.generation,t,self.now,self.now))
        deadline=self.c.tasks['a']['yaw_reference_end_stamp']
        self.assertAlmostEqual(deadline-self.now,math.radians(2)/CONFIG['control']['yaw_rate_rad_s'])
        t=SimpleNamespace(id=2,start=self.now+.2,duration=1.)
        self.assertTrue(self.c.trajectory_received(self.c.generation,t,self.now+.2,self.now+.2))
        self.assertEqual(self.c.tasks['a']['yaw_reference_end_stamp'],deadline)

    def test_init_does_not_reinitialize_active_navigation(self):
        self.nav();generation=self.c.generation
        with self.assertRaises(Rejected):self.cmd('init',flight_authorized=True)
        self.assertEqual(generation,self.c.generation)

    def test_relative_uses_body_yaw_at_bridge_acceptance(self):
        self.c.pose[3]=math.pi/2
        self.cmd('relative',task_id='rel',relative=[1,-.5,.2,-.3],localization_epoch=self.c.epoch)
        np.testing.assert_allclose(self.c.tasks['rel']['goal'],[.5,1,1.2,math.pi/2-.3])

    def test_future_replan_keeps_current_reference_until_start(self):
        self.nav()
        old=SimpleNamespace(start=self.now-1,duration=10,id=1,
                            sample=lambda t:(np.array([.2,0,1]),np.zeros(3),np.zeros(3)))
        new=SimpleNamespace(start=self.now+.2,duration=10,id=2,
                            sample=lambda t:(np.array([.3,0,1]),np.zeros(3),np.zeros(3)))
        self.c.trajectory=old
        self.c.planner_at=self.now
        self.c.trajectory_received(self.c.generation,new,self.now,self.now)
        np.testing.assert_allclose(self.c.tick(self.now,self.now)[0],[.2,0,1])
        np.testing.assert_allclose(self.c.tick(self.now+.2,self.now+.2)[0],[.3,0,1])

    def test_failed_task_cannot_resume_until_measured_stop(self):
        self.nav();self.c.fail('planner failure')
        with self.assertRaises(Rejected):self.nav('b')
        self.stable();self.nav('b')
        self.assertEqual(self.c.tasks['a']['status'],'failed')

    def delayed_takeoff(self):
        self.c.update_state(True,False,False,'POSCTL',self.now)
        self.c.manual=False;self.c.enabled=False
        self.cmd('init',flight_authorized=True)
        self.cmd('takeoff',task_id='up',auto_arm=True,localization_epoch=self.c.epoch)
        self.c.tasks['up']['auto_start_pending']=False

    def test_takeoff_spool_delay_bounds_lead_then_arrives(self):
        self.delayed_takeoff()
        for _ in range(80):
            self.feed(dt=.05);self.cmd('heartbeat')
            out=self.c.tick(self.now,self.now)
            self.assertLessEqual(out[0][2]-self.c.pose[2],.200001)
            self.assertNotEqual(self.c.tasks['up']['status'],'failed')
        z=1.
        for _ in range(300):
            out=self.c.tick(self.now,self.now)
            z+=min(.015,max(0.,out[0][2]-z)*.4)
            self.feed(p=(0,0,z),dt=.05);self.cmd('heartbeat')
            if self.c.tasks['up']['status']=='arrived':break
        self.assertEqual(self.c.tasks['up']['status'],'arrived')

    def test_takeoff_stall_fails_without_relaxing_tracking_limit(self):
        self.delayed_takeoff()
        for _ in range(220):
            self.feed(dt=.05);self.cmd('heartbeat');self.c.tick(self.now,self.now)
            if self.c.tasks['up']['status']=='failed':break
        self.assertEqual(self.c.tasks['up']['error'],'takeoff has no measured upward progress')
        self.assertEqual(self.c.c['max_tracking_error_m'],.5)

    def test_takeoff_waits_for_pilot_and_cancel_cannot_arm(self):
        self.c.update_state(True,False,False,'POSCTL',self.now)
        # A fresh ground bridge, before any OFFBOARD authority was acquired.
        self.c.manual=False;self.c.enabled=False
        self.cmd('init',flight_authorized=True)
        self.cmd('takeoff',task_id='up',goal=[0,0,2,0],localization_epoch=self.c.epoch)
        np.testing.assert_allclose(self.c.tick(self.now,self.now)[0],[0,0,1])
        self.cmd('cancel',task_id='up')
        self.assertIsNone(self.c.generation)
        self.assertFalse(self.c.armed)

    def test_init_reseeds_old_yaw_and_reset_discards_it(self):
        self.c.yaw=-1.5
        self.cmd('init',flight_authorized=True)
        self.assertEqual(self.c.yaw,self.c.pose[3])
        self.c.reset()
        self.assertIsNone(self.c.yaw)

    def test_software_takeoff_blocks_ascent_until_confirmed(self):
        self.c.update_state(True,False,False,'POSCTL',self.now)
        self.c.manual=False;self.c.enabled=False
        self.cmd('init',flight_authorized=True)
        self.cmd('takeoff',task_id='up',auto_arm=True,localization_epoch=self.c.epoch)
        self.c.update_state(True,True,False,'OFFBOARD',self.now)
        self.c.tick(self.now,self.now)
        self.now+=.05
        self.assertEqual(self.c.tick(self.now,self.now)[0][2],1.)
        self.c.tasks['up']['auto_start_pending']=False
        self.now+=.05
        self.assertGreater(self.c.tick(self.now,self.now)[0][2],1.)

    def test_manual_takeover_during_landing(self):
        self.nav();self.cmd('land',task_id='land')
        self.c.update_state(True,True,True,'AUTO.LAND',self.now)
        self.c.update_state(True,True,True,'POSCTL',self.now)
        self.assertTrue(self.c.manual)
        self.assertEqual(self.c.tasks['land']['status'],'failed')
        self.assertIsNone(self.c.tick(self.now,self.now))

    def test_startup_grace_and_executing_heartbeat_loss(self):
        self.nav()
        json.dumps(self.c.tasks,allow_nan=False)
        for _ in range(12): self.feed()
        self.assertEqual(self.c.status(self.now)['planner_state'],'starting')
        self.assertFalse(self.c.status(self.now)['planner_ok'])
        self.assertTrue(self.c.status(self.now)['hold_ready'])
        np.testing.assert_allclose(self.c.tick(self.now,self.now)[0],[0,0,1])
        self.c.planner_heartbeat(self.c.generation,self.now)
        self.assertEqual(self.c.status(self.now)['planner_state'],'ready')
        self.c.tasks['a']['status']='executing'
        for _ in range(12): self.feed()
        out=self.c.tick(self.now,self.now)
        self.assertEqual(self.c.tasks['a']['status'],'failed')
        self.assertIn('heartbeat lost',self.c.tasks['a']['error'])
        np.testing.assert_allclose(out[0],[0,0,1])

    def test_startup_timeout_and_old_heartbeat_fence(self):
        self.nav();old=self.c.generation
        self.cmd('cancel',task_id='a');self.stable();self.nav('b')
        self.assertFalse(self.c.planner_heartbeat(old,self.now))
        self.assertEqual(self.c.planner_state(self.now),'starting')
        for i in range(102):
            self.feed()
            if i%4==0:self.cmd('heartbeat')
            self.c.tick(self.now,self.now)
        self.assertEqual(self.c.tasks['b']['status'],'failed')
        self.assertIn('heartbeat',self.c.tasks['b']['error'])

    def test_land_requires_fresh_explicit_ground_and_disarm(self):
        self.nav();self.cmd('land',task_id='land')
        self.c.update_state(True,False,False,'AUTO.LAND',self.now)
        for value in [0,3,4]:
            self.c.update_extended_state(value,self.now)
            self.assertNotEqual(self.c.tasks['land']['status'],'arrived')
        self.c.update_state(True,True,False,'AUTO.LAND',self.now)
        self.c.update_extended_state(1,self.now)
        self.assertNotEqual(self.c.tasks['land']['status'],'arrived')
        self.now+=2.1
        self.c.update_state(True,False,False,'AUTO.LAND',self.now)
        self.assertNotEqual(self.c.tasks['land']['status'],'arrived')
        self.c.update_extended_state(1,self.now)
        self.assertEqual(self.c.tasks['land']['status'],'arrived')

    def test_landing_disarm_and_mode_exit_with_explicit_ground(self):
        self.nav();self.cmd('land',task_id='land')
        self.c.update_state(True,True,True,'AUTO.LAND',self.now)
        self.c.update_extended_state(1,self.now)
        self.c.update_state(True,False,False,'POSCTL',self.now)
        self.assertEqual(self.c.tasks['land']['status'],'arrived')
        self.assertFalse(self.c.manual)

    def test_ground_with_stale_disarm_does_not_complete_land(self):
        self.nav();self.cmd('land',task_id='land')
        self.c.update_state(True,False,False,'AUTO.LAND',self.now)
        self.now+=2.1
        self.c.update_extended_state(1,self.now)
        self.assertNotEqual(self.c.tasks['land']['status'],'arrived')
        self.c.update_state(True,False,False,'AUTO.LAND',self.now)
        self.assertEqual(self.c.tasks['land']['status'],'arrived')


class PlannerHandshakeTest(unittest.TestCase):
    def test_private_log_confirmation_requires_exact_state_and_map(self):
        planner=PlannerProcess.__new__(PlannerProcess)
        planner.goal=[1,0,1,0];planner.sent=False;planner.fsm_ready=False;planner.map_observed=False
        planner.state_tail='';planner.milestones={}
        planner.process=SimpleNamespace(poll=lambda:None)
        sent=[]
        planner.goal_pub=SimpleNamespace(get_num_connections=lambda:1,publish=sent.append)
        planner.state_log=io.StringIO('[FSM]Drone:0, from INIT to WAIT_TARGET\n')
        with patch.dict(sys.modules,{'quadrotor_msgs.msg':SimpleNamespace(GoalSet=lambda **kw:kw)}):
            planner.poll();self.assertFalse(planner.fsm_ready)
            planner.state_tail='';planner.state_log=io.StringIO('[FSM]Drone:0, from WAIT_TARGET to GEN_NEW_TRAJ\n')
            planner.milestones['first_odom_forwarded']=time.monotonic()
            planner.poll();self.assertFalse(planner.fsm_ready)
            planner.state_log=io.StringIO('[FSM]Drone:0, from INIT to WAIT_TARGET\n')
            planner.poll();self.assertTrue(planner.fsm_ready)
            self.assertIn('fsm_log_confirmation',planner.milestones)
            self.assertEqual(sent,[])
            planner.map_received(None);planner.poll();planner.poll()
            self.assertEqual(sent,[dict(drone_id=0,goal=[1,0,1])])

    def test_closed_planner_ignores_late_odometry_without_touching_handles(self):
        planner=PlannerProcess.__new__(PlannerProcess)
        planner.callback_lock=threading.Lock();planner.closed=True
        planner.odometry('late sample')

    def test_goal_waits_for_observed_fsm_and_map_not_elapsed_sleep(self):
        planner=PlannerProcess.__new__(PlannerProcess)
        planner.callback_lock=threading.Lock();planner.closed=False;planner.state_log=None
        sent=[];odom=[]
        planner.goal=[1,0,1,0];planner.sent=False;planner.fsm_ready=False;planner.map_observed=False
        planner.milestones={};planner.birth=time.monotonic()-100
        planner.process=SimpleNamespace(poll=lambda:None)
        planner.goal_pub=SimpleNamespace(get_num_connections=lambda:1,publish=sent.append)
        planner.odom_pub=SimpleNamespace(get_num_connections=lambda:1,publish=odom.append)
        planner.data_sub=SimpleNamespace(get_num_connections=lambda:0)
        with patch.dict(sys.modules,{'quadrotor_msgs.msg':SimpleNamespace(GoalSet=lambda **kw:kw)}):
            planner.poll();self.assertEqual(sent,[])
            planner.odometry('sample');self.assertEqual(odom,[])
            planner.data_sub.get_num_connections=lambda:1
            planner.odometry('sample');self.assertEqual(odom,['sample'])
            planner.initialized(None);planner.poll();self.assertEqual(sent,[])
            planner.map_received(SimpleNamespace(width=0,height=1));planner.poll();planner.poll()
            self.assertEqual(sent,[dict(drone_id=0,goal=[1,0,1])])


class ObservationTest(unittest.TestCase):
    def test_full_attitude_translation_and_interpolation(self):
        q=[math.sin(.2),0,0,math.cos(.2)]
        hist=[dict(stamp=t,frame='world',body='base_link',xyz=[t,2,3],q=q) for t in [1.,1.04]]
        tr,sync,_,_=interpolate_pose(hist,1.02,.05)
        np.testing.assert_allclose(tr[:3,3],[1.02,2,3])
        np.testing.assert_allclose(tr[:3,:3],rotation(q))
        self.assertAlmostEqual(sync,.02)
        with self.assertRaises(ValueError):interpolate_pose(hist,2,.05)
        with self.assertRaises(ValueError):interpolate_pose(hist,1.02,.01)

    def test_dynamic_gimbal_support_stamps_are_bounded(self):
        edges={('base_link','gimbal'):[1.,1.04],('gimbal','optical'):None}
        self.assertAlmostEqual(transform_sync_error(edges,'base_link','optical',1.02,.05),.02)
        edges[('base_link','gimbal')]=[0.,2.]
        with self.assertRaises(ValueError):transform_sync_error(edges,'base_link','optical',1.,.05)

    def test_observation_resize_full_extrinsic_and_missing_calibration(self):
        hw=OwlEgoHardware(copy.deepcopy(CONFIG))
        hw.c['hardware'].update(intrinsics_mode='camera_info',extrinsics_mode='tf',fixed_camera_pitch_deg=None)
        hw.c['hardware']['camera_optical_frame']='optical'
        hw.c['hardware']['long_edge_px']=320
        stamp=SimpleNamespace(to_sec=lambda:1.02,to_nsec=lambda:1020000000)
        hw.ros=SimpleNamespace(Time=SimpleNamespace(now=lambda:SimpleNamespace(to_sec=lambda:1.05)),Duration=lambda t:t)
        hw.image=SimpleNamespace(header=SimpleNamespace(stamp=stamp,frame_id='optical'),width=640,height=480)
        hw.info=SimpleNamespace(header=SimpleNamespace(frame_id='optical'),width=640,height=480,
                K=[500.,0,320.,0,500.,240.,0,0,1],D=[0.]*5,distortion_model='plumb_bob',
                binning_x=0,binning_y=0,roi=SimpleNamespace(width=0,height=0))
        q=[math.sin(.2),0,0,math.cos(.2)]
        hw.history.extend(dict(stamp=t,frame='world',body='base_link',xyz=[1,2,3],q=q) for t in [1.,1.04])
        hw.tf_edges={('base_link','optical'):None}
        tf=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda:0)),
            transform=SimpleNamespace(translation=SimpleNamespace(x=.1,y=.2,z=.3),
            rotation=SimpleNamespace(x=0,y=0,z=0,w=1)))
        hw.tf=SimpleNamespace(lookup_transform=lambda *args:tf)
        hw.cv=SimpleNamespace(imgmsg_to_cv2=lambda *args:np.zeros((480,640,3),np.uint8))
        obs=build_observation(hw)
        self.assertEqual(obs['image_size'],[320,240])
        np.testing.assert_allclose(obs['intrinsics'],[[250,0,160],[0,250,120],[0,0,1]])
        tr=np.array(obs['world_from_camera_optical_cm'])
        np.testing.assert_allclose(tr[:3,3],100*(np.array([1,2,3])+rotation(q)@np.array([.1,.2,.3])))
        self.assertEqual(obs['frame_id'],build_observation(hw)['frame_id'])
        hw.c['hardware']['extrinsics_mode']='body_coincident_fixed'
        optical_to_body=np.array([[0,0,1],[-1,0,0],[0,-1,0.]])
        hw.c['hardware']['body_from_camera_optical_rotation']=optical_to_body.tolist()
        hw.c['hardware']['camera_optical_frame']=''
        hw.tf_edges={}
        hw.tf=SimpleNamespace(lookup_transform=lambda *args: self.fail('fixed extrinsics must not query TF'))
        fixed=build_observation(hw)
        np.testing.assert_allclose(np.array(fixed['world_from_camera_optical_cm'])[:3,3],[100,200,300])
        np.testing.assert_allclose(np.array(fixed['world_from_camera_optical_cm'])[:3,:3],rotation(q)@optical_to_body)
        hw.c['hardware']['body_from_camera_optical_rotation']=None
        with self.assertRaisesRegex(ValueError,'explicit proper'):build_observation(hw)
        hw.info.K=[0.]*9
        with self.assertRaises(ValueError):build_observation(hw)
        hw.c['hardware'].update(intrinsics_mode='approximate_fov',assumed_horizontal_fov_deg=90.,fixed_camera_pitch_deg=20.)
        hw.info=None
        approximate=build_observation(hw)
        self.assertFalse(approximate['rectified'])
        self.assertEqual(approximate['calibration_quality'],'approximate')
        np.testing.assert_allclose(approximate['intrinsics'],[[160,0,160],[0,160,120],[0,0,1]],atol=1e-8)
        np.testing.assert_allclose(np.array(approximate['world_from_camera_optical_cm'])[:3,3],[100,200,300])

    def test_approximate_intrinsics_without_camera_info(self):
        k,d,rectified,meta=camera_intrinsics(dict(intrinsics_mode='approximate_fov',assumed_horizontal_fov_deg=90.),
                                           SimpleNamespace(width=1280,height=720),None)
        np.testing.assert_allclose(k,[[640,0,640],[0,640,360],[0,0,1]])
        self.assertIsNone(d);self.assertFalse(rectified)
        self.assertEqual(meta['distortion'],'unknown_not_corrected')
        for value in [None,True,0,180,float('nan')]:
            with self.assertRaises(ValueError):camera_intrinsics(dict(intrinsics_mode='approximate_fov',assumed_horizontal_fov_deg=value),SimpleNamespace(width=1280,height=720),None)

    def test_existing_agent_strict_default_rejects_approximation(self):
        from robot_client.owl_ego import decode_observation
        with self.assertRaisesRegex(ValueError,'rectified'):
            decode_observation(dict(ok=True,rectified=False,calibration_quality='approximate'))

    def test_fixed_pitch_and_optical_axes(self):
        r=fixed_optical_rotation(dict(fixed_camera_pitch_deg=20.))
        np.testing.assert_allclose(r@np.array([0,0,1]),[math.cos(math.radians(20)),0,-math.sin(math.radians(20))])
        np.testing.assert_allclose(r.T@r,np.eye(3),atol=1e-9)

    def test_coordinate_roundtrip(self):
        p=[1,2,3,.7]
        np.testing.assert_allclose(enu_pose(public_pose(p),30),p)
        with self.assertRaises(ApiError):enu_pose(dict(x=0,y=0,z=math.inf,yaw=0),30)

    def camera_cache(self):
        hw=OwlEgoHardware(copy.deepcopy(CONFIG))
        hw.now_s=lambda:1.06
        def image(t):return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda:t)))
        hw.image=image(1.04)
        hw.history.append(dict(stamp=1.033,frame='world',body='base_link',xyz=[0,0,1],q=[0,0,0,1]))
        return hw,image

    def test_camera_wait_releases_lock_and_recovers(self):
        hw,_=self.camera_cache()
        def feed():
            time.sleep(.02)
            with hw.camera_changed:
                hw.history.append(dict(hw.history[-1],stamp=1.066))
                hw.camera_changed.notify_all()
        t=threading.Thread(target=feed);t.start()
        frame,_,hist,_,_=hw.camera_snapshot();t.join()
        self.assertEqual(frame.header.stamp.to_sec(),1.04)
        self.assertAlmostEqual(interpolate_pose(hist,1.04,.05)[1],.026)
        for _ in range(20):self.assertIs(hw.camera_snapshot()[0],frame)

    def test_camera_cached_exposure_and_stale_unavailable(self):
        hw,image=self.camera_cache()
        old=image(1.02);hw.images.extend([old,hw.image])
        hw.history.appendleft(dict(hw.history[0],stamp=1.0))
        self.assertIs(hw.camera_snapshot()[0],old)
        hw.now_s=lambda:2.
        start=time.monotonic()
        with self.assertRaises(ValueError) as cm:hw.camera_snapshot()
        self.assertEqual(cm.exception.error_code,'observation_unavailable')
        self.assertTrue(cm.exception.retryable)
        self.assertLess(time.monotonic()-start,.15)

    def test_camera_epoch_change_during_wait_is_not_retryable(self):
        hw,_=self.camera_cache()
        def reset():
            time.sleep(.02)
            with hw.camera_changed:
                hw.epoch='new';hw.images.clear();hw.history.clear();hw.image=None
                hw.camera_changed.notify_all()
        t=threading.Thread(target=reset);t.start()
        with self.assertRaises(ValueError) as cm:hw.camera_snapshot()
        t.join()
        self.assertEqual(cm.exception.error_code,'localization_epoch_changed')
        self.assertFalse(cm.exception.retryable)

    def test_camera_permanently_missing_odom_is_bounded(self):
        hw,_=self.camera_cache()
        for _ in range(2):
            with self.assertRaises(ValueError) as cm:hw.camera_snapshot()
            self.assertEqual(cm.exception.code,503)


class FakeHardware:
    epoch='e'
    def __init__(self):
        self.session=None;self.commands=[];self.tasks={};self.block=threading.Event()
    def start(self):pass
    def close(self):pass
    def snapshot(self):
        return dict(session_id=self.session,pose=[0,0,1,math.pi/2],tasks=copy.deepcopy(self.tasks),
                    health=dict(odom_ok=True,localization_epoch='e',control_ready=True))
    def observation(self):raise ValueError('calibration absent')
    def command(self,op,data):
        self.commands.append((op,data))
        if op=='acquire':
            if self.session:return dict(ok=False,error='busy')
            self.session=data['session_id']
        if op=='release':self.session=None
        if op in ('navigate','relative'):self.tasks[data['task_id']]=dict(task_id=data['task_id'],status='executing',stopped=False)
        if op=='cancel':self.tasks[data['task_id']].update(status='cancelled',stopped=True)
        return dict(ok=True,**({'task_id':data['task_id']} if 'task_id' in data else {}))


class HttpTest(unittest.TestCase):
    def setUp(self):
        self.hw=FakeHardware();self.c=OwlEgoController(hardware=self.hw,config=CONFIG)
        self.server=run_http_server(self.c,NullKeepalive(),'127.0.0.1',0)
        self.url='http://127.0.0.1:'+str(self.server.server_port)
        self.sid=self.rpc('POST','/v21/session',dict(request_id=str(uuid.uuid4())))[1]['session_id']
    def tearDown(self):self.server.shutdown();self.server.server_close()
    def rpc(self,method,path,data=None):
        req=urllib.request.Request(self.url+path,data=json.dumps(data).encode() if data is not None else None,method=method)
        try:
            with urllib.request.build_opener(urllib.request.ProxyHandler({})).open(req,timeout=3) as r:return r.status,json.load(r)
        except urllib.error.HTTPError as r:return r.code,json.load(r)
    def body(self,**kw):return dict(session_id=self.sid,request_id=str(uuid.uuid4()),**kw)
    def test_capabilities_missing_observation_and_methods(self):
        self.assertEqual(self.rpc('GET','/v21/capabilities')[1]['protocol_version'],1)
        self.assertNotEqual(self.rpc('GET','/v21/observation')[0],200)
        self.assertEqual(self.rpc('GET','/takeoff')[0],405)
        self.assertEqual(self.rpc('POST','/velocity',{})[0],404)
    def test_idempotency_and_mismatch(self):
        body=self.body(localization_epoch='e',pose=dict(x=0,y=100,z=100,yaw=0))
        a=self.rpc('POST','/v21/navigation',body)
        self.assertEqual(a,self.rpc('POST','/v21/navigation',body))
        self.assertEqual(sum(op=='navigate' for op,d in self.hw.commands),1)
        body['pose']['x']=10
        self.assertEqual(self.rpc('POST','/v21/navigation',body)[0],409)
    def test_blocking_relative_allows_heartbeat_status_and_cancel(self):
        data=self.body(x=-100,y=0,z=0,yaw=30,timeout_s=2)
        results=[]
        t=threading.Thread(target=lambda:results.append(self.rpc('POST','/move_relative_xyz_yaw',data)))
        t.start()
        for _ in range(100):
            if self.hw.tasks:break
            time.sleep(.01)
        tid=next(iter(self.hw.tasks))
        relative=next(d['relative'] for op,d in self.hw.commands if op=='relative')
        np.testing.assert_allclose(relative,[-1,0,0,-math.radians(30)],atol=1e-8)
        started=time.monotonic()
        self.assertEqual(self.rpc('POST','/v21/heartbeat',dict(session_id=self.sid))[0],200)
        self.assertEqual(self.rpc('GET','/v21/navigation/status?task_id='+tid)[0],200)
        self.assertEqual(self.rpc('POST','/v21/navigation/cancel',self.body(task_id=tid))[0],200)
        self.assertLess(time.monotonic()-started,1)
        t.join(2);self.assertFalse(t.is_alive());self.assertNotEqual(results[0][0],200)
    def test_observation_retryable_error_is_machine_readable(self):
        from robot.controllers.owl_ego_observation import ObservationUnavailable, ObservationEpochChanged, InvalidObservation
        for error,code,retryable in [(ObservationUnavailable('wait'),503,True),
                                    (ObservationEpochChanged('epoch'),409,False),
                                    (InvalidObservation('bad calibration'),422,False)]:
            def fail():raise error
            self.c.observation=fail
            status,value=self.rpc('GET','/v21/observation')
            self.assertEqual(status,code)
            self.assertEqual(value['retryable'],retryable)
            self.assertEqual(value['error_code'],error.error_code)

    def test_malformed_json_and_mutation_query_do_not_bypass_session(self):
        req=urllib.request.Request(self.url+'/init',data=b'{invalid',method='POST')
        with self.assertRaises(urllib.error.HTTPError) as cm:
            urllib.request.build_opener(urllib.request.ProxyHandler({})).open(req,timeout=2)
        self.assertEqual(cm.exception.code,400)
        self.assertFalse(json.load(cm.exception)['ok'])
        data=dict(session_id='bad',request_id=str(uuid.uuid4()))
        self.assertEqual(self.rpc('POST','/init?session_id='+self.sid,data)[0],409)

    def test_unknown_task_and_foreign_session(self):
        self.assertEqual(self.rpc('GET','/v21/navigation/status?task_id=unknown')[0],404)
        self.assertEqual(self.rpc('POST','/init',dict(session_id='bad',request_id=str(uuid.uuid4())))[0],409)


class LegacyCompatibilityTest(unittest.TestCase):
    def test_old_init_health_and_no_v21_capabilities(self):
        class Legacy:
            def init(self):return dict(ok=True,message='legacy init')
            def health(self):return dict(initialized=True)
        server=run_http_server(Legacy(),NullKeepalive(),'127.0.0.1',0)
        opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
        url='http://127.0.0.1:'+str(server.server_port)
        try:
            with opener.open(url+'/init') as r:self.assertEqual(json.load(r)['message'],'legacy init')
            with opener.open(url+'/health') as r:self.assertTrue(json.load(r)['health']['initialized'])
            with self.assertRaises(urllib.error.HTTPError) as cm:opener.open(url+'/v21/capabilities')
            self.assertEqual(cm.exception.code,404)
        finally:
            server.shutdown();server.server_close()


if __name__=='__main__':unittest.main()
