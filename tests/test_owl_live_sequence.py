"""HTTP-only live-runner regression tests: no ROS and no real flight."""
import importlib.util
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

spec=importlib.util.spec_from_file_location('live_sequence',Path(__file__).resolve().parents[1]/'scripts/owl_ego/live_sequence.py')
live=importlib.util.module_from_spec(spec);spec.loader.exec_module(live)


class LiveTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.runner=live.Runner(live.parse_args(['--output',self.temp.name]))
        self.addCleanup(self.runner.log.close)

    def test_preview_over_http_has_no_mutations(self):
        calls=[]
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*args):pass
            def do_GET(self):
                calls.append(('GET',self.path))
                values={
                    '/v21/capabilities':dict(ok=True,backend='owl_ego',protocol_version=1,async_navigation=True,cancel_and_hold=True,control_lease=True,relative_xyz_yaw=True),
                    '/health':dict(ok=True,health=dict(localization_epoch='e',odom_ok=True,stopped=True,landed_state=1,landed_state_fresh=True)),
                    '/get_pose':dict(ok=True,localization_epoch='e',pose=dict(x=0,y=0,z=-10,yaw=0)),
                    '/motion_tolerances':dict(ok=True,motion_tolerances=dict(position_tolerance_cm=15,yaw_tolerance_deg=5))}
                self.send_response(200);self.end_headers();self.wfile.write(json.dumps(values[self.path]).encode())
            def do_POST(self):
                calls.append(('POST',self.path));self.send_error(500)
        server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
        worker=threading.Thread(target=server.serve_forever,daemon=True);worker.start()
        try:
            self.runner.a.url='http://127.0.0.1:'+str(server.server_port)
            self.runner.run()
        finally:server.shutdown();server.server_close();worker.join()
        self.assertTrue(calls);self.assertTrue(all(method=='GET' for method,path in calls))
        self.assertFalse(json.loads(self.runner.summary.read_text())['executed'])

    def test_preview_mutation_guard(self):
        with self.assertRaisesRegex(live.Failure,'Preview'):
            self.runner.rpc('POST','/init',{})

    def test_relative_uses_current_body_heading(self):
        p=live.offset(dict(x=10,y=20,z=100,yaw=90),30,10,5,20)
        self.assertAlmostEqual(p['x'],0);self.assertAlmostEqual(p['y'],50)
        self.assertEqual(p['z'],105);self.assertEqual(p['yaw'],110)

    def test_health_starting_allowed_but_epoch_and_takeover_abort(self):
        h=dict(localization_epoch='e',odom_ok=True,initialized=True,airborne=True,control_ready=True,hold_ready=True,planner_state='starting')
        self.runner.epoch='e';self.runner.rpc=lambda *a,**kw:dict(health=h)
        self.runner.health()
        h['localization_epoch']='new'
        with self.assertRaisesRegex(live.Failure,'epoch'):self.runner.health()
        h['localization_epoch']='e';h['manual_takeover']=True
        with self.assertRaises(live.Failure):self.runner.health()

    def test_cancel_acceptance_is_not_stop_confirmation(self):
        states=iter([dict(status='stopping',stopped=False),dict(status='cancelled',stopped=True)])
        self.runner.health=lambda:None;self.runner.task=lambda tid:next(states)
        with patch.object(live.time,'sleep'):
            result=self.runner.wait_task('a',('cancelled',))
        self.assertTrue(result['stopped'])

    def test_failure_cleanup_waits_before_release_and_never_lands(self):
        self.runner.sid='s';order=[]
        def rpc(method,path,data=None):
            order.append(path)
            return dict(health=dict(active_task_id='a')) if path=='/health' else dict(ok=True)
        self.runner.rpc=rpc
        self.runner.post=lambda path,**kw:order.append(path)
        self.runner.wait_task=lambda *a,**kw:order.append('confirmed-stop')
        self.runner.cleanup()
        self.assertEqual(order,['/health','/v21/navigation/cancel','confirmed-stop','/v21/session/release'])

    def test_manual_takeover_cleanup_does_not_cancel_or_land(self):
        self.runner.sid='s';calls=[]
        self.runner.rpc=lambda method,path,data=None: calls.append(path) or dict(health=dict(active_task_id='a',manual_takeover=True))
        self.runner.cleanup()
        self.assertEqual(calls,['/health','/v21/session/release'])

    def test_blocking_motion_does_not_block_safety_monitor(self):
        gate=threading.Event();self.addCleanup(gate.set)
        self.runner.rpc=lambda *a,**kw: gate.wait(1) or dict(ok=True)
        self.runner.health=lambda **kw: (_ for _ in ()).throw(live.Failure('epoch changed'))
        begin=time.monotonic()
        with self.assertRaisesRegex(live.Failure,'epoch changed'):
            self.runner.blocking('/move_relative_xyz_yaw',18,x=30)
        gate.set();self.assertLess(time.monotonic()-begin,.2)

    def test_heartbeat_failure_propagates(self):
        self.runner.rpc=lambda *a,**kw: (_ for _ in ()).throw(live.Failure('network lost'))
        self.runner.heartbeat()
        self.assertIn('network lost',self.runner.hb_error)

    def test_step_prompt_keeps_heartbeat_running(self):
        self.runner.a.step=True
        self.runner.sid='s'
        beats=[]
        self.runner.rpc=lambda *a,**kw: beats.append(time.monotonic()) or dict(ok=True)
        self.runner.health=lambda:None
        worker=threading.Thread(target=self.runner.heartbeat)
        worker.start()
        try:
            with patch('builtins.input',side_effect=lambda prompt:time.sleep(1.05) or ''):
                self.runner.stage('stable-step')
        finally:
            self.runner.stop.set();worker.join(2)
        self.assertGreaterEqual(len(beats),3)

    def test_cleanup_failure_is_not_reported_as_confirmed(self):
        self.runner.sid='s'
        def rpc(method,path,data=None):
            if path=='/health':return dict(health=dict(active_task_id=None))
            raise live.Failure('network unavailable')
        self.runner.rpc=rpc
        self.assertFalse(self.runner.cleanup())

    def test_parameter_validation_rejects_nonfinite_and_wrong_origin(self):
        for args in [['--b-forward-cm','nan'],['--delay-s','100'],['--url','file:///tmp/a'],['--url','http://host/path']]:
            with self.assertRaises(SystemExit):live.parse_args(args)


if __name__=='__main__':unittest.main()
