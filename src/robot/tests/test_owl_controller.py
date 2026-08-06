import math
import sys
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np


SRC_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SRC_ROOT))

from robot.controllers.owl import OwlController  # noqa: E402
from robot_client.owl import OwlClient  # noqa: E402
from robot.server import NullKeepalive, run_http_server  # noqa: E402


class FakeOwlHardware:
    def __init__(self):
        self.connected = False
        yaw = math.pi / 2.0
        self.pose = {
            "x_m": 0.0,
            "y_m": 0.0,
            "z_m": 1.0,
            "yaw_rad": yaw,
            "orientation_xyzw": (
                0.0,
                0.0,
                math.sin(yaw / 2.0),
                math.cos(yaw / 2.0),
            ),
            "frame_id": "world",
        }
        self.controls = []
        self.goals = []
        frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        frame[:, :, 1] = 127
        ok, encoded = cv2.imencode(".jpg", frame)
        assert ok
        self.rgb = encoded.tobytes()

    def connect(self):
        self.connected = True
        return {"ok": True, "message": "fake connected"}

    def health(self):
        return {
            "initialized": self.connected,
            "airborne": self.connected,
            "control_ready": self.connected,
            "connected": self.connected,
            "armed": self.connected,
            "mode": "OFFBOARD",
            "landed_state": 2,
            "odom_ok": True,
            "rgb_ok": True,
            "battery_ok": True,
        }

    def get_pose_ros(self):
        return dict(self.pose)

    def get_compressed_rgb(self):
        return self.rgb, "jpeg"

    def publish_control(self, cmd, drone_id):
        self.controls.append((cmd, drone_id))

    def publish_goal(self, x_m, y_m, z_m, orientation_xyzw, frame_id):
        self.goals.append((x_m, y_m, z_m, orientation_xyzw, frame_id))
        self.pose.update(x_m=x_m, y_m=y_m, z_m=z_m)


class OwlControllerTest(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.hardware = FakeOwlHardware()
        self.controller = OwlController(
            image_dir=self.temp_dir.name,
            hardware=self.hardware,
            position_tolerance_cm=1.0,
            position_stable_samples=1,
            position_poll_hz=100.0,
            navigation_start_delay_s=0.0,
        )
        self.controller.init()

    def tearDown(self):
        self.temp_dir.cleanup()

    def test_pose_converts_ros_enu_to_agent_convention(self):
        pose = self.controller.get_pose()["pose"]
        self.assertAlmostEqual(pose["x"], 0.0)
        self.assertAlmostEqual(pose["y"], 0.0)
        self.assertAlmostEqual(pose["z"], 100.0)
        self.assertAlmostEqual(pose["yaw"], -90.0)

    def test_xyz_goal_uses_forward_and_right_axes_and_starts_105_once(self):
        self.controller.move_relative_xyz(x=100, y=0, z=0)
        self.assertAlmostEqual(self.hardware.goals[0][0], 0.0, places=6)
        self.assertAlmostEqual(self.hardware.goals[0][1], 1.0, places=6)
        self.assertEqual(self.hardware.controls, [(105, 0)])

        self.controller.move_relative_xyz(x=0, y=100, z=0)
        self.assertAlmostEqual(self.hardware.goals[1][0], 1.0, places=6)
        self.assertAlmostEqual(self.hardware.goals[1][1], 1.0, places=6)
        self.assertEqual(self.hardware.controls, [(105, 0)])

    def test_rgb_is_resized_and_encoded_as_jpeg(self):
        raw = self.controller.get_rgb_byte()
        frame = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
        self.assertIsNotNone(frame)
        self.assertEqual(frame.shape[:2], (480, 640))

    def test_nonzero_yaw_fails_explicitly(self):
        with self.assertRaises(NotImplementedError):
            self.controller.rotate(10)
        with self.assertRaises(NotImplementedError):
            self.controller.move_relative_xyz_yaw(0, 0, 0, 10)


class OwlHttpIntegrationTest(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.hardware = FakeOwlHardware()
        self.controller = OwlController(
            image_dir=self.temp_dir.name,
            hardware=self.hardware,
            position_tolerance_cm=1.0,
            position_stable_samples=1,
            position_poll_hz=100.0,
            navigation_start_delay_s=0.0,
        )
        self.server = run_http_server(
            self.controller,
            NullKeepalive(),
            "127.0.0.1",
            0,
        )
        self.port = self.server.server_address[1]
        self.client = OwlClient("127.0.0.1", self.port, 5.0)

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.temp_dir.cleanup()

    def test_start_capture_pose_and_xyz_over_http(self):
        start = self.client.start()
        self.assertTrue(start["ok"])
        frame = self.client.capture(include_depth=False)
        self.assertEqual(frame.shape, (480, 640, 3))

        result = self.client.move_relative(dx=100, dy=0, dz=0, dyaw=0)
        self.assertTrue(result["ok"])
        pose = self.client.get_pose()
        self.assertAlmostEqual(pose["y"], -100.0, places=5)
        self.assertEqual(self.hardware.controls, [(105, 0)])


if __name__ == "__main__":
    unittest.main()
