from contextlib import closing

import rospy
from geometry_msgs.msg import PoseStamped


WAYPOINT_TOPIC = "/buaa_goal"
POLL_INTERVAL_S = 0.1


class NextWaypoint:
    def __init__(self):
        self.positions = (None, None)
        self.subscriber = rospy.Subscriber(
            WAYPOINT_TOPIC,
            PoseStamped,
            self.update,
            queue_size=1,
        )

    def connected(self):
        return self.subscriber.get_num_connections() > 0

    def update(self, message):
        p = message.pose.position
        position = (p.x, p.y, p.z)
        _, current = self.positions
        if position != current:
            self.positions = (current, position)

    def get(self):
        """Return a paired snapshot of the previous and next route waypoints."""
        previous, position = self.positions
        if position is None:
            raise RuntimeError(f"No waypoint received from {WAYPOINT_TOPIC}")
        return {
            "position_m": list(position),
            "previous_position_m": None if previous is None else list(previous),
        }

    def close(self):
        self.subscriber.unregister()
        self.positions = (None, None)


def test():
    """Print previous and next route waypoints whenever the cache changes."""
    rospy.init_node("test_next_waypoint")
    with closing(NextWaypoint()) as waypoint:
        print(f"Waiting for waypoints from {WAYPOINT_TOPIC}...", flush=True)
        previous_sample = None
        while not rospy.is_shutdown():
            if waypoint.positions[1] is not None:
                sample = waypoint.get()
                if sample != previous_sample:
                    print(
                        f"Previous waypoint: {sample['previous_position_m']}; "
                        f"next waypoint: {sample['position_m']}",
                        flush=True,
                    )
                    previous_sample = sample
            rospy.sleep(POLL_INTERVAL_S)


if __name__ == "__main__":
    try:
        test()
    except (KeyboardInterrupt, rospy.ROSInterruptException):
        pass
