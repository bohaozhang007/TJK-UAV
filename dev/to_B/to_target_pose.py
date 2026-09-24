import argparse
import time
from contextlib import closing

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped


GOAL_TOPIC = "/initialpose"
CONNECTION_TIMEOUT_S = 5.0
POLL_INTERVAL_S = 0.1


class TargetPose:
    def __init__(self):
        self.publisher = rospy.Publisher(
            GOAL_TOPIC,
            PoseWithCovarianceStamped,
            queue_size=1,
        )

    def connected(self):
        return self.publisher.get_num_connections() > 0

    def send(self, pose):
        goal = PoseWithCovarianceStamped()
        goal.header.frame_id = pose["frame_id"]
        goal.header.stamp = rospy.Time.now()
        p = goal.pose.pose.position
        p.x, p.y, p.z = pose["position_m"]
        # Custom mission protocol: orientation.w carries yaw in degrees, not quaternion w.
        goal.pose.pose.orientation.w = pose["yaw_deg"]
        self.publisher.publish(goal)

    def close(self):
        self.publisher.unregister()


def test(pose=None):
    """Send one real target pose; the mission controller will acquire control."""
    if pose is None:
        parser = argparse.ArgumentParser(description="Send a target position and yaw")
        parser.add_argument(
            "--position",
            nargs=3,
            type=float,
            required=True,
            help="SLAM x y z in metres",
        )
        parser.add_argument(
            "--yaw",
            type=float,
            required=True,
            help="SLAM yaw in degrees",
        )
        parser.add_argument("--frame", default="world")
        args = parser.parse_args(rospy.myargv()[1:])
        pose = {
            "frame_id": args.frame,
            "position_m": args.position,
            "yaw_deg": args.yaw,
        }
    rospy.init_node("test_to_target_pose")
    with closing(TargetPose()) as target:
        deadline = time.monotonic() + CONNECTION_TIMEOUT_S
        while not target.connected():
            if rospy.is_shutdown():
                return
            if time.monotonic() >= deadline:
                raise RuntimeError("Target pose topic has no subscriber")
            rospy.sleep(POLL_INTERVAL_S)
        if not rospy.is_shutdown():
            target.send(pose)
            rospy.sleep(POLL_INTERVAL_S)
            print("Target pose sent.", flush=True)


if __name__ == "__main__":
    test()
