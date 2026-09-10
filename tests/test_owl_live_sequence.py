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

    def test_track_left_and_right_are_anchored_to_saved_P_despite_arrival_error(self):
        r=self.runner
        P=dict(x=200,y=50,z=100,yaw=90)
        # Actual poses have admissible errors; neither XY nor yaw errors may
        # redefine the second destination. Only the final turn is body-relative.
        samples=iter([dict(x=202,y=51,z=110,yaw=92),
                      dict(x=302,y=51,z=110,yaw=92),
                      dict(x=302,y=51,z=110,yaw=92),
                      dict(x=102,y=51,z=110,yaw=92),
                      dict(x=102,y=51,z=110,yaw=92),
                      dict(x=102,y=51,z=110,yaw=-178)])
        r.pose=lambda:next(samples)
        r.stage=lambda _:None;r.wait_motion_ready=lambda:None
        r.tolerances=dict(position_tolerance_cm=15,yaw_tolerance_deg=5)
        r.wait_task=lambda tid:dict(task_id=tid,status='arrived',stopped=True)
        with patch.object(r,'navigate',return_value='nav') as nav,patch.object(r,'blocking',return_value=dict(task_id='yaw')) as turn:
            r.track(P)
        goals=[c.args[0] for c in nav.call_args_list]
        self.assertEqual(len(goals),2)
        self.assertAlmostEqual(goals[0]['x'],300)
        self.assertAlmostEqual(goals[1]['x'],100)
        self.assertAlmostEqual(live.distance(goals[0],goals[1]),200)
        for goal in goals:
            self.assertAlmostEqual(goal['y'],50)
            self.assertEqual(goal['z'],100);self.assertEqual(goal['yaw'],90)
        turn.assert_called_once_with('/move_relative_xyz_yaw',18,x=0,y=0,z=0,yaw=90,timeout_s=15)

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

    def test_robot_execution_error_is_reported_without_claiming_authority_loss(self):
        h=dict(odom_ok=True,initialized=True,airborne=True,control_ready=True,hold_ready=True,
               planner_state='not_required',error='trajectory expired before measured arrival')
        self.runner.rpc=lambda *a,**kw:dict(health=h)
        with self.assertRaisesRegex(live.Failure,'Robot motion failed: trajectory expired'):
            self.runner.health()

    def test_only_explicit_landing_monitor_can_ignore_localization_reset(self):
        r=self.runner;r.epoch='old'
        h=dict(localization_epoch='new',odom_ok=False,landing=True,manual_takeover=False)
        r.rpc=lambda *a,**kw:dict(health=h)
        r.health(flight=False,require_localization=False)
        with self.assertRaisesRegex(live.Failure,'epoch'):r.health(flight=False)
        h['manual_takeover']=True
        with self.assertRaises(live.Failure):r.health(flight=False,require_localization=False)
        h['manual_takeover']=False;r.hb_error='lease lost'
        with self.assertRaisesRegex(live.Failure,'Heartbeat'):r.health(flight=False,require_localization=False)

    def test_successful_landing_resets_client_epoch_for_next_ground_init(self):
        r=self.runner;r.epoch='old';r.a.execute=True
        h=dict(localization_epoch='new',odom_ok=False,landing=True)
        r.rpc=lambda method,*a,**kw:dict(health=h) if method=='GET' else dict(ok=True)
        r.blocking('/land',2,flight=False)
        self.assertIsNone(r.epoch)

    def test_navigation_waits_for_current_stop_then_submits_once(self):
        order=[]
        states=iter([False,False,True])
        def health():
            value=next(states);order.append(value)
            return dict(stopped=value,active_task_id=None)
        self.runner.health=health
        self.runner.post=lambda *a,**kw:order.append('submit') or dict(task_id='b')
        with patch.object(live.time,'sleep'):
            self.assertEqual(self.runner.navigate(dict(x=2,y=0,z=1,yaw=0)),'b')
        self.assertEqual(order,[False,False,True,'submit'])

    def test_stop_timeout_submits_no_relative_motion(self):
        self.runner.health=lambda:dict(stopped=False,stop_diagnostics=dict(speed_m_s=.12))
        clock=[0.]
        with patch.object(live.time,'monotonic',side_effect=lambda:clock[0]),patch.object(live.time,'sleep',side_effect=lambda _:clock.__setitem__(0,clock[0]+9)),patch.object(self.runner,'record'),patch.object(self.runner,'rpc') as rpc:
            with self.assertRaisesRegex(live.Failure,'停止超时'):
                self.runner.blocking('/move_relative_xyz_yaw',18,x=30)
            rpc.assert_not_called()

    def test_cancel_wait_reports_current_diagnostics_and_keeps_checking_health(self):
        clock=[0.]
        self.runner.health=lambda:dict(stop_diagnostics=dict(speed_m_s=.15,yaw_rate_deg_s=.2,stable_duration_s=0.))
        states=iter([dict(task_id='a',status='stopping'),dict(task_id='a',status='stopping'),dict(task_id='a',status='cancelled',stopped=True)])
        self.runner.task=lambda _:next(states)
        with patch.object(live.time,'monotonic',side_effect=lambda:clock[0]),patch.object(live.time,'sleep',side_effect=lambda _:clock.__setitem__(0,clock[0]+2.1)),patch.object(self.runner,'record') as record:
            result=self.runner.wait_task('a',('cancelled',))
        self.assertTrue(result['stopped'])
        reports=[c for c in record.call_args_list if c.args[0]=='stop_wait_progress']
        self.assertEqual(len(reports),2)
        self.assertEqual(reports[0].kwargs['diagnostics']['speed_m_s'],.15)

    def test_cancel_wait_times_out_at_eight_seconds_without_new_motion(self):
        clock=[0.]
        self.runner.health=lambda:dict(stop_diagnostics=dict(method='pose_window',position_span_m=.2))
        self.runner.task=lambda _:dict(task_id='a',status='stopping',stopped=False)
        with patch.object(live.time,'monotonic',side_effect=lambda:clock[0]),patch.object(live.time,'sleep',side_effect=lambda _:clock.__setitem__(0,clock[0]+1)),patch.object(self.runner,'record'),patch.object(self.runner,'post') as post:
            with self.assertRaisesRegex(live.Failure,'confirmation timed out'):
                self.runner.wait_task('a',('cancelled',))
            self.assertEqual(clock[0],8.)
            post.assert_not_called()

    def test_stop_wait_rechecks_health_and_does_not_queue_active_task(self):
        states=iter([dict(stopped=False),dict(stopped=True,active_task_id='a')])
        self.runner.health=lambda:next(states)
        with patch.object(live.time,'sleep'),patch.object(self.runner,'post') as post:
            with self.assertRaisesRegex(live.Failure,'still active'):self.runner.navigate({})
            post.assert_not_called()

    def test_stop_wait_aborts_on_authority_loss(self):
        with patch.object(self.runner,'health',side_effect=[dict(stopped=False),live.Failure('Localization epoch changed')]),patch.object(live.time,'sleep'),patch.object(self.runner,'post') as post:
            with self.assertRaisesRegex(live.Failure,'epoch'):self.runner.navigate({})
            post.assert_not_called()

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

class ConsoleTest(unittest.TestCase):
    def setUp(self):
        import sys
        sys.modules['live_sequence']=live
        spec=importlib.util.spec_from_file_location('owl_console',Path(__file__).resolve().parents[1]/'scripts/owl_ego/console.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup)
        self.console=module.Console(live.parse_args(['--execute','--output',temp.name]))
        self.addCleanup(self.console.r.log.close)

    def test_start_and_help_submit_nothing(self):
        with patch.object(self.console.r,'rpc') as rpc:
            self.console.command('help')
            rpc.assert_not_called()
        self.assertIsNone(self.console.r.sid)

    def test_takeoff_requires_init(self):
        with self.assertRaises(live.Failure):self.console.command('takeoff')

    def test_relative_commands_require_init_and_reject_invalid_arguments(self):
        with self.assertRaisesRegex(live.Failure,'init'):
            self.console.command('move_rel_xyz_yaw 50 0 0 0')
        with patch.object(self.console,'launch') as launch:
            for line in ['move_rel_xyz_yaw 1 2 3','move_rel_xyz 1 2 3 4',
                         'move_rel_xyz_yaw 0.5 0 0 0','move_rel_xyz_yaw nan 0 0 0']:
                with self.assertRaises(live.Failure):self.console.command(line)
            launch.assert_not_called()

    def test_relative_aliases_preserve_signed_integer_units(self):
        c=self.console;c.r.sid='s'
        with patch.object(c,'move_relative') as move:
            for line,values in [('move_rel_xyz_yaw 50 -20 0 -90',[50,-20,0,-90]),
                                ('move_relative_xyz_yaw 0 0 -10 30',[0,0,-10,30]),
                                ('move_rel_xyz 0 50 0',[0,50,0,0]),
                                ('move_relative_xyz -50 0 0',[-50,0,0,0])]:
                c.command(line);c.command('wait');move.assert_called_with(values)

    def test_relative_does_not_queue_behind_an_active_console_task(self):
        self.console.r.sid='s'
        with patch.object(self.console,'worker') as worker,patch.object(self.console,'move_relative') as move:
            worker.is_alive.return_value=True
            with self.assertRaisesRegex(live.Failure,'任务运行中'):
                self.console.command('move_rel_xyz_yaw 50 0 0 0')
            move.assert_not_called()

    def test_relative_reports_transient_peak_even_when_final_height_returns(self):
        c=self.console;r=c.r;order=[]
        poses=[dict(x=0,y=0,z=z,yaw=0) for z in (100,112,101)]
        def blocking(path,timeout,**kw):
            self.assertEqual(path,'/move_relative_xyz_yaw')
            self.assertEqual(timeout,18)
            callback=kw.pop('on_poll')
            self.assertEqual(kw,dict(x=50,y=0,z=0,yaw=0,timeout_s=15))
            callback();return dict(task_id='t')
        with patch.object(r,'wait_motion_ready',side_effect=lambda:order.append('ready')), \
             patch.object(r,'pose',side_effect=lambda:order.append('pose') or poses.pop(0)), \
             patch.object(r,'blocking',side_effect=blocking),patch.object(r,'wait_task',return_value=dict(status='arrived')):
            c.move_relative([50,0,0,0])
        self.assertEqual(order[0],'ready')
        events=[json.loads(s) for s in Path(r.log.name).read_text().splitlines()]
        summary=next(e for e in events if e['event']=='relative_motion_summary')
        self.assertTrue(summary['completed']);self.assertEqual(summary['delta_z_cm'],1)
        self.assertEqual(summary['max_rise_cm'],12)

    def test_relative_failure_preserves_diagnostic_without_claiming_completion(self):
        r=self.console.r
        with patch.object(r,'wait_motion_ready'),patch.object(r,'pose',return_value=dict(x=0,y=0,z=100,yaw=0)), \
             patch.object(r,'blocking',side_effect=live.Failure('motion failed')):
            with self.assertRaisesRegex(live.Failure,'motion failed'):
                self.console.move_relative([50,0,0,0])
        events=[json.loads(s) for s in Path(r.log.name).read_text().splitlines()]
        self.assertFalse(events[-1]['completed'])

    def test_abort_blocks_future_motion_but_allows_cancel(self):
        r=self.console.r;r.abort.set()
        with patch.object(live.Runner,'rpc',return_value={}) as rpc:
            for path in ['/init','/takeoff','/v21/navigation','/move_relative_xyz_yaw']:
                with self.assertRaises(live.Failure):r.rpc('POST',path,{})
            rpc.assert_not_called()
            r.rpc('POST','/v21/navigation/cancel',{})
            rpc.assert_called_once()

    def test_init_after_landing_retires_old_session_and_initializes_new(self):
        c=self.console;r=c.r;r.sid='old'
        h=dict(stopped=True,active_task_id=None,initialized=False,airborne=False,
               landed_state=1,landed_state_fresh=True,localization_epoch='e')
        def rpc(method,path,data=None,timeout=2):
            if path=='/v21/capabilities':return dict(backend='owl_ego',software_takeoff=True)
            if path=='/motion_tolerances':return dict(motion_tolerances={})
            if path=='/v21/session':return dict(session_id='new')
            raise AssertionError(path)
        with patch.object(r,'rpc',side_effect=rpc),patch.object(r,'health',return_value=h), \
             patch.object(r,'cleanup',return_value=True) as cleanup,patch.object(r,'heartbeat'),patch.object(r,'post') as post:
            c.command('init');r.hb_thread.join(1)
            cleanup.assert_called_once();post.assert_called_once_with('/init')
            self.assertEqual(r.sid,'new')

    def test_repeat_init_when_ready_does_not_reset_or_acquire(self):
        c=self.console;r=c.r;r.sid='old'
        with patch.object(r,'rpc',return_value=dict(backend='owl_ego',software_takeoff=True)), \
             patch.object(r,'health',return_value=dict(stopped=True,initialized=True)),patch.object(r,'post') as post:
            c.command('init');post.assert_not_called()
            self.assertEqual(r.sid,'old')

    def test_takeoff_and_land_share_session(self):
        c=self.console;c.r.sid='existing'
        with patch.object(c.r,'blocking',return_value={}) as blocking:
            c.command('takeoff');c.command('wait')
            blocking.assert_called_with('/takeoff',65,flight=False,auto_arm=True)
            c.command('land');c.command('wait')
            blocking.assert_called_with('/land',95,flight=False)
        self.assertEqual(c.r.sid,'existing')
