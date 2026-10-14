import rospy
from geometry_msgs.msg import PoseStamped


WAYPOINT_TOPIC = "/buaa_goal"


class NextWaypoint:
    def __init__(self):
        self.position = None
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
        self.position = (p.x, p.y, p.z)

    def get(self):
        """Return a snapshot of the next route waypoint."""
        position = self.position
        if position is None:
            raise RuntimeError(f"No waypoint received from {WAYPOINT_TOPIC}")
        return {"position_m": list(position)}

    def close(self):
        self.subscriber.unregister()
        self.position = None
