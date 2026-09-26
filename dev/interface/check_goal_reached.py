import threading
import time
from contextlib import closing

import rospy
from std_msgs.msg import Bool


# The mission controller publishes True once per reached pose; it does not publish False.
GOAL_REACHED_TOPIC = "/vln_point_reached"
ARRIVAL_TIMEOUT_S = 120.0
POLL_INTERVAL_S = 0.1


class GoalReached:
    def __init__(self):
        self.arrived = threading.Event()
        self.subscriber = rospy.Subscriber(
            GOAL_REACHED_TOPIC,
            Bool,
            self.update,
            queue_size=10,
        )

    def connected(self):
        return self.subscriber.get_num_connections() > 0

    def reset(self):
        self.arrived.clear()

    def update(self, message):
        if message.data:
            self.arrived.set()

    def wait(self, timeout):
        return self.arrived.wait(timeout)

    def close(self):
        self.subscriber.unregister()
        self.reset()


def test(timeout=ARRIVAL_TIMEOUT_S):
    """Start before sending a target; wait for its single True arrival message."""
    rospy.init_node("test_check_goal_reached")
    with closing(GoalReached()) as feedback:
        feedback.reset()
        print("Waiting for goal arrival feedback: True...", flush=True)
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown():
            if time.monotonic() >= deadline:
                raise RuntimeError(f"Goal arrival feedback timeout of {timeout:g} s exceeded")
            if feedback.wait(POLL_INTERVAL_S):
                print("Goal reached.", flush=True)
                return True
        return False


if __name__ == "__main__":
    test()
