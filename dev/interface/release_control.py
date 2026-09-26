import time
from contextlib import closing

import rospy
from std_msgs.msg import Bool


RELEASE_TOPIC = "/recover_mission"
CONNECTION_TIMEOUT_S = 5.0
POLL_INTERVAL_S = 0.1


class ControlRelease:
    def __init__(self):
        self.publisher = rospy.Publisher(
            RELEASE_TOPIC,
            Bool,
            queue_size=1,
        )

    def connected(self):
        return self.publisher.get_num_connections() > 0

    def send(self):
        self.publisher.publish(Bool(data=True))

    def close(self):
        self.publisher.unregister()


def test():
    """Send one real control-release command to the mission controller."""
    rospy.init_node("test_release_control")
    with closing(ControlRelease()) as release:
        deadline = time.monotonic() + CONNECTION_TIMEOUT_S
        while not release.connected():
            if rospy.is_shutdown():
                return
            if time.monotonic() >= deadline:
                raise RuntimeError(f"Control release connection timeout of {CONNECTION_TIMEOUT_S:g} s exceeded")
            rospy.sleep(POLL_INTERVAL_S)
        if not rospy.is_shutdown():
            release.send()
            rospy.sleep(POLL_INTERVAL_S)
            print("Control release sent.", flush=True)


if __name__ == "__main__":
    test()
