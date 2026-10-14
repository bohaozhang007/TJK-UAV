import rospy
from geometry_msgs.msg import PoseStamped


WAYPOINT_TOPIC = "/buaa_goal"


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
