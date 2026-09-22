import contextlib
import io
import json
import os
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, Mock, patch

from app.robot.i7.bringup.stack_supervisor import main, recovery_plan
from app.robot.i7.bringup.recover_startup import require_ground, require_no_control


class RecoveryGuardTests(unittest.TestCase):
    def test_missing_ground_disarmed_or_fresh_feedback_blocks_recovery(self):
        for armed, landed, age, connected in [(False, 1, .1, True), (True, 1, .1, True),
                                             (False, 2, .1, True), (False, 1, 3., True),
                                             (False, 1, .1, False)]:
            state = SimpleNamespace(connected=connected, armed=armed, mode='POSCTL', header=Mock())
            ground = SimpleNamespace(landed_state=landed, header=Mock())
            ros = MagicMock()
            ros.wait_for_message.side_effect = [state, ground]
            ros.Time.now.return_value.__sub__.return_value.to_sec.return_value = age
            messages = SimpleNamespace(State=object, ExtendedState=SimpleNamespace(LANDED_STATE_ON_GROUND=1))
            with self.subTest(armed=armed, landed=landed, age=age, connected=connected), \
                 patch.dict('sys.modules', {'rospy': ros, 'mavros_msgs.msg': messages}):
                if not armed and landed == 1 and age < 1 and connected:
                    require_ground()
                else:
                    with self.assertRaises(RuntimeError):
                        require_ground()

    def test_active_control_blocks_recovery(self):
        master = Mock()
        for topic, node in [('/health', '/i7_ego_v22_bridge'),
                            ('/mavros/setpoint_raw/local', '/other_controller')]:
            master.getSystemState.return_value = [[(topic, [node])], [], []]
            with self.assertRaises(RuntimeError):
                require_no_control(master)


class RecoveryPlanTests(unittest.TestCase):
    def test_alignment_reboots_px4_after_lio_cleanup(self):
        self.assertEqual(recovery_plan({'errors': ['lio_px4_alignment: mismatch']}),
                         ['stop_lio', 'reboot_px4'])

    def test_stale_odom_restarts_lidar_chain_without_px4_reboot(self):
        self.assertEqual(recovery_plan({'errors': ['odom: stale', 'cloud: stale']}),
                         ['stop_lio', 'stop_livox'])

    def test_camera_only_retries_clean_stack(self):
        self.assertEqual(recovery_plan({'errors': ['camera_baseline: timeout']}), [])

    def test_configuration_ownership_and_missing_report_never_restart(self):
        for errors in [[], ['planner: wrong hash'], ['motion_publishers: active'],
                       ['component: unknown'], ['camera_baseline: timeout', 'sensor_geometry: invalid']]:
            with self.subTest(errors=errors), self.assertRaises(RuntimeError):
                recovery_plan({'errors': errors})


class SupervisorTests(unittest.TestCase):
    def simulate(self, mode):
        calls = []
        def popen(command, env=None, **kwargs):
            calls.append(command)
            child = Mock(pid=12345)
            child.poll.return_value = 1
            if env is not None:
                Path(env['I7_STARTUP_REPORT']).write_text(json.dumps({'errors': ['camera_baseline: timeout']}))
                if mode == 'runtime':
                    Path(env['I7_STARTUP_READY']).touch()
                child.wait.return_value = 130 if mode == 'interrupt' else 1
            else:
                child.wait.return_value = 0
            return child
        previous = os.getcwd()
        with tempfile.TemporaryDirectory() as folder:
            os.chdir(folder)
            try:
                with patch('sys.argv', ['stack', '--attempts', '3']), \
                     patch('app.robot.i7.bringup.stack_supervisor.fcntl.flock'), \
                     patch('app.robot.i7.bringup.stack_supervisor.signal.signal'), \
                     patch('app.robot.i7.bringup.stack_supervisor.subprocess.Popen', side_effect=popen), \
                     contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                    result = main()
            finally:
                os.chdir(previous)
        return result, calls

    def test_retry_budget_is_finite(self):
        status, calls = self.simulate('startup')
        self.assertEqual(status, 1)
        self.assertEqual(len(calls), 5)  # Three startup attempts, two recoveries.

    def test_successful_startup_disables_recovery_on_later_failure(self):
        status, calls = self.simulate('runtime')
        self.assertEqual(status, 1)
        self.assertEqual(len(calls), 1)

    def test_interrupt_never_retries(self):
        status, calls = self.simulate('interrupt')
        self.assertEqual(status, 130)
        self.assertEqual(len(calls), 1)


if __name__ == '__main__':
    unittest.main()
