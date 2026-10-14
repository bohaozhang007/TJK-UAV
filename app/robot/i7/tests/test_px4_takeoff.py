"""Flight-free tests of native takeoff ownership, handoff and default behavior."""
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'app/robot/owl_ego/ros/src'))
from app.robot.config_loader import load_robot_config
from owl_nav_v22.core import FlightCore
from owl_nav_v22.px4_takeoff import prepare_px4


class WorkflowTests(unittest.TestCase):
    def run_workflow(self, *, reject=None, stuck=False, revoke=False):
        state = dict(mode='POSCTL', armed=False, airborne=False, stopped=False, height_ok=False)
        calls, clock = [], [0.]
        def guard():
            if revoke and state['armed']:
                raise RuntimeError('ownership revoked')
            return state.copy()
        def mode(value):
            calls.append(value)
            if reject == value:
                raise RuntimeError('rejected')
            state['mode'] = value
        def arm():
            calls.append('arm')
            state['armed'] = True
        def sleep(dt):
            clock[0] += dt
            if state['armed'] and not stuck:
                state.update(mode='AUTO.LOITER', airborne=True, stopped=True, height_ok=True)
        try:
            prepare_px4(guard, mode, arm, lambda: True, calls.append,
                        timeout=.2, clock=lambda: clock[0], sleep=sleep)
        finally:
            self.calls = calls
        return state

    def test_native_ascent_then_offboard(self):
        self.assertEqual(self.run_workflow()['mode'], 'OFFBOARD')
        self.assertEqual(self.calls, ['AUTO.TAKEOFF', 'PX4_TAKEOFF', 'arm', 'HANDOFF', 'OFFBOARD'])

    def test_rejection_never_arms(self):
        with self.assertRaisesRegex(RuntimeError, 'rejected'):
            self.run_workflow(reject='AUTO.TAKEOFF')
        self.assertEqual(self.calls, ['AUTO.TAKEOFF'])

    def test_no_arrival_does_not_handoff_or_retry(self):
        with self.assertRaisesRegex(RuntimeError, 'timed out'):
            self.run_workflow(stuck=True)
        self.assertEqual(self.calls.count('arm'), 1)
        self.assertNotIn('OFFBOARD', self.calls)

    def test_revoked_ownership_never_handoffs(self):
        with self.assertRaisesRegex(RuntimeError, 'revoked'):
            self.run_workflow(revoke=True)
        self.assertNotIn('OFFBOARD', self.calls)

    def test_handoff_rejection_not_success(self):
        with self.assertRaisesRegex(RuntimeError, 'rejected'):
            self.run_workflow(reject='OFFBOARD')
        self.assertEqual(self.calls.count('OFFBOARD'), 1)


class CoreTests(unittest.TestCase):
    def core(self, native=True):
        config = load_robot_config('i7')['control']
        config['px4_takeoff_land'] = native
        c = FlightCore(config, manual_offboard_takeoff=not native)
        c.update_extended_state(1, 0.)
        c.update_state(True, False, False, 'POSCTL', 0.)
        c.odometry([0., 0., .2], 0., [0., 0., 0.], 0., 0., 'world', 0.)
        c.command('acquire', dict(session_id='s', operator_token='o'), 0.)
        c.command('init', dict(session_id='s', flight_authorized=True), 0.)
        c.command('takeoff', dict(session_id='s', task_id='t', localization_epoch=c.epoch,
                                 auto_arm=True, px4_takeoff_height_m=2.), 0.)
        c.tasks['t']['takeoff_phase'] = 'PREPARING'
        return c

    def test_defaults_and_bool_validation(self):
        config = load_robot_config('i7')['control']
        self.assertIs(config['px4_takeoff_land'], False)
        FlightCore(config)
        with tempfile.TemporaryDirectory() as d:
            p = Path(d)/'bad.yaml'
            p.write_text('control:\n  px4_takeoff_land: "false"\n')
            with self.assertRaisesRegex(ValueError, 'boolean'):
                load_robot_config('i7', p)

    def test_native_uses_fcu_height_and_measured_stream(self):
        c = self.core()
        self.assertAlmostEqual(c.tasks['t']['goal'][2], 2.2)
        c.update_state(True, False, False, 'AUTO.TAKEOFF', .1)
        c.tasks['t']['takeoff_phase'] = 'PX4_TAKEOFF'
        c.update_state(True, True, True, 'AUTO.TAKEOFF', .2)
        c.odometry([.2, .1, .6], 0., [0., 0., 0.], 0., .2, 'world', .2)
        output = c.tick(.2, .2)
        self.assertEqual(output[0].tolist(), [.2, .1, .6])
        self.assertEqual(c.active, 't')
        self.assertIsNone(c.last_error)

    def test_pilot_takeover_revokes_native_task(self):
        c = self.core()
        c.update_state(True, False, False, 'AUTO.TAKEOFF', .1)
        c.tasks['t']['takeoff_phase'] = 'PX4_TAKEOFF'
        c.update_state(True, True, True, 'POSCTL', .2)
        self.assertTrue(c.manual)
        self.assertFalse(c.enabled)
        self.assertEqual(c.tasks['t']['status'], 'failed')
        self.assertIsNone(c.tick(.2, .2))

    def test_offboard_before_handoff_is_rejected(self):
        c = self.core()
        c.update_state(True, True, True, 'OFFBOARD', .1)
        self.assertTrue(c.manual)
        self.assertEqual(c.tasks['t']['status'], 'failed')

    def test_ack_pending_cannot_finish_on_odometry(self):
        c = self.core()
        c.tasks['t']['takeoff_phase'] = 'HANDOFF'
        c.update_state(True, True, True, 'OFFBOARD', .1)
        c.pose[2] = 2.2
        for i in range(1, 9):
            c.odometry([0., 0., 2.2], 0., [0., 0., 0.], 0., i*.1, 'world', i*.1)
        self.assertEqual(c.active, 't')
        c.tasks['t']['auto_start_pending'] = False
        c.odometry([0., 0., 2.2], 0., [0., 0., 0.], 0., .9, 'world', .9)
        self.assertEqual(c.tasks['t']['status'], 'arrived')

    def test_land_preempts_native_takeoff(self):
        c = self.core()
        c.tasks['t']['takeoff_phase'] = 'PX4_TAKEOFF'
        c.update_state(True, True, True, 'AUTO.TAKEOFF', .1)
        c.command('land', dict(session_id='s', task_id='land'), .1)
        c.update_state(True, True, True, 'AUTO.TAKEOFF', .2)
        self.assertFalse(c.manual)
        c.update_state(True, True, True, 'AUTO.LAND', .3)
        self.assertFalse(c.enabled)
        c.update_extended_state(1, .4)
        c.update_state(True, False, False, 'AUTO.LAND', .4)
        self.assertEqual(c.tasks['land']['status'], 'arrived')

    def test_default_still_streams_existing_ground_hold(self):
        c = self.core(native=False)
        self.assertFalse(c.tasks['t']['px4_takeoff'])
        self.assertEqual(c.tick(.1, .1)[0].tolist(), [0., 0., .2])


if __name__ == '__main__':
    unittest.main()
