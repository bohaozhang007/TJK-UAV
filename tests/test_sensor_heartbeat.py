"""Offline liveness checks; never connects to ROS."""
import importlib.util
from pathlib import Path
import struct
import unittest

spec=importlib.util.spec_from_file_location('sensor_heartbeat',Path(__file__).resolve().parents[1]/'scripts/owl_ego/sensor_heartbeat.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def frame(stamp):
    sec=int(stamp)
    return struct.pack('<IIII',0,sec,int((stamp-sec)*1e9),0)+b'payload'


class HeartbeatTest(unittest.TestCase):
    def setUp(self):
        self.s=m.Stream('/camera','sensor_msgs/Image',0.)

    def snap(self,now,ros=None):
        return self.s.snapshot(now,now if ros is None else ros,2.,2.,1.)['state']

    def feed(self,now,stamp=None):
        self.s.receive(frame(now if stamp is None else stamp),'sensor_msgs/Image',now)

    def test_wait_no_data_recovery_and_disconnect(self):
        self.assertEqual(self.snap(.2),'WAITING')
        self.assertEqual(self.snap(2.),'NO_DATA')
        for n in range(10):self.feed(2.1+n*.1)
        self.assertEqual(self.snap(3.),'OK')
        self.assertEqual(self.snap(6.),'NO_DATA')
        for n in range(10):self.feed(6.1+n*.1)
        self.assertEqual(self.snap(7.),'OK')

    def test_repeated_old_or_future_timestamps_are_not_healthy(self):
        for n in range(10):self.feed(10+n*.1,1.)
        self.assertEqual(self.snap(11.),'STALE_STAMP')
        self.feed(12.,20.)
        self.assertEqual(self.snap(12.),'STALE_STAMP')

    def test_wall_clock_detects_frozen_ros_clock(self):
        self.feed(1.,100.)
        self.feed(5.,100.)
        self.assertEqual(self.snap(5.,100.),'FROZEN_STAMP')

    def test_type_malformed_payload_and_low_rate(self):
        self.s.receive(frame(2.),'wrong_type',2.)
        self.assertEqual(self.snap(2.),'INVALID')
        self.s.receive(b'bad','sensor_msgs/Image',3.)
        self.assertEqual(self.snap(3.),'INVALID')
        self.feed(4.)
        self.assertEqual(self.snap(4.),'LOW_RATE')

    def test_status_write_is_small_latest_snapshot(self):
        import tempfile,json
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'status.json'
            m.write_status(p,dict(healthy=True))
            m.write_status(p,dict(healthy=False))
            self.assertFalse(json.loads(p.read_text())['healthy'])
            self.assertEqual(list(Path(d).iterdir()),[p])


if __name__=='__main__':unittest.main()
