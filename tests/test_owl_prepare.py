"""Controller handover tests; no ROS master or real FCU access."""
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('prepare', Path(__file__).resolve().parents[1]/'scripts/owl_ego/prepare.py')
prepare = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prepare)


class PrepareTest(unittest.TestCase):
    def setUp(self):
        self.nodes = {'/captain', '/mavros_controller', '/mavros', '/fast_lio', '/camera'}
        self.extra = []
        self.state = dict(fresh=True, connected=True, armed=False, landed_state=1)
        self.killed = []
        self.now = 0.

    def graph(self):
        pubs = [('/mavros/setpoint_raw/local', ['/mavros_controller'])] if '/mavros_controller' in self.nodes else []
        pubs += [('/mavros/setpoint_raw/target_local', ['/mavros'])]
        if self.extra:
            pubs.append(('/mavros/setpoint_position/local', self.extra))
        return pubs, [], [('/loggers', list(self.nodes))]

    def kill(self, node):
        self.killed.append(node)
        self.nodes.remove(node)

    def sleep(self, dt):
        self.now += dt

    def run_handover(self, shutdown=None):
        return prepare.handover(self.graph, lambda: self.state, shutdown or self.kill,
                                clock=lambda: self.now, sleep=self.sleep)

    def test_known_controllers_stop_and_other_nodes_survive_with_idempotent_retry(self):
        self.assertEqual(self.run_handover(), ['/captain', '/mavros_controller'])
        self.assertEqual(self.nodes, {'/mavros', '/fast_lio', '/camera'})
        self.assertGreaterEqual(self.now, 1.)
        self.assertEqual(self.run_handover(), [])

    def test_unsafe_or_missing_telemetry_never_stops_anything(self):
        for change in [dict(fresh=False), dict(connected=False), dict(armed=True),
                       dict(armed=None), dict(landed_state=0), dict(landed_state=2)]:
            with self.subTest(change=change):
                self.setUp(); self.state.update(change)
                with self.assertRaises(RuntimeError): self.run_handover()
                self.assertEqual(self.killed, [])

    def test_unknown_controller_or_running_owl_bridge_blocks_all_mutations(self):
        for node in ['/owl_nav', '/other_controller']:
            self.extra = [node]
            with self.assertRaisesRegex(RuntimeError, '其他运动发布者'): self.run_handover()
            self.assertEqual(self.killed, [])

    def test_arming_between_shutdowns_prevents_second_shutdown(self):
        def kill(node):
            self.kill(node); self.state['armed'] = True
        with self.assertRaisesRegex(RuntimeError, '未解锁'): self.run_handover(kill)
        self.assertEqual(self.killed, ['/captain'])

    def test_reappearing_controller_times_out_without_repeated_killing(self):
        def kill(node):
            self.kill(node); self.nodes.add(node)
        with self.assertRaisesRegex(RuntimeError, '再次启动'): self.run_handover(kill)
        self.assertEqual(self.killed, ['/captain', '/mavros_controller'])
        self.assertLess(self.now, 5.2)

    def test_shutdown_failure_prevents_continuing(self):
        def kill(node):
            raise RuntimeError('shutdown failed')
        with self.assertRaisesRegex(RuntimeError, 'shutdown failed'): self.run_handover(kill)
        self.assertIn('/mavros_controller', self.nodes)

    def test_new_unknown_controller_during_verification_is_rejected(self):
        def kill(node):
            self.kill(node)
            if node == '/mavros_controller': self.extra = ['/other_controller']
        with self.assertRaisesRegex(RuntimeError, '其他运动发布者'): self.run_handover(kill)


if __name__ == '__main__':
    unittest.main()
