"""Normalize FAST-LIO2 odometry and publish K40T frames with receipt timestamps."""
import argparse
import copy
import os
import threading
import time

import cv2
import numpy as np

from app.robot.config_loader import load_robot_config
from app.robot.i7.calibration import validate


class Sensors:
    def __init__(self, config):
        import rospy
        from cv_bridge import CvBridge
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Image
        self.ros, self.c, self.cv = rospy, config, CvBridge()
        self.previous = None
        self.odom_pub = rospy.Publisher(config['topics']['odom'], Odometry, queue_size=5)
        self.rgb_pub = rospy.Publisher(config['topics']['rgb'], Image, queue_size=1)
        self.sub = rospy.Subscriber(config['source']['odom'], Odometry, self.odometry, queue_size=5)
        self.done = threading.Event()
        self.worker = threading.Thread(target=self.camera, daemon=True)
        self.worker.start()
        rospy.on_shutdown(self.close)

    def odometry(self, message):
        from tf.transformations import quaternion_matrix, quaternion_from_matrix
        try:
            if (message.header.frame_id != self.c['source']['world_frame']
                    or message.child_frame_id != self.c['source']['body_frame']):
                raise ValueError('FAST-LIO2 frames differ from configured virtual body reference')
            p, q = message.pose.pose.position, message.pose.pose.orientation
            quaternion = np.array([q.x, q.y, q.z, q.w])
            if not np.isfinite([p.x, p.y, p.z, *quaternion]).all() or abs(np.linalg.norm(quaternion)-1) > .02:
                raise ValueError('invalid FAST-LIO2 pose')
            stamp = message.header.stamp.to_sec()
            if not 0 <= self.ros.Time.now().to_sec()-stamp <= self.c['control']['odom_timeout_s']:
                raise ValueError('stale FAST-LIO2 pose')
            # Vendor odometry already uses the rotated Livox IMU body frame.
            world_body = quaternion_matrix(quaternion)
            world_body[:3, 3] = [p.x, p.y, p.z]
            previous, self.previous = self.previous, (stamp, world_body)
            if previous is None:
                return
            dt = stamp-previous[0]
            if not 0 < dt <= self.c['control']['odom_timeout_s']:
                raise ValueError('FAST-LIO2 timestamp discontinuity')
            # FAST-LIO2 builds may omit twist; derive it from stamped body poses.
            velocity = world_body[:3, :3].T @ ((world_body[:3, 3]-previous[1][:3, 3])/dt)
            delta = previous[1][:3, :3].T @ world_body[:3, :3]
            angular = cv2.Rodrigues(delta)[0].reshape(3)/dt
            out = copy.deepcopy(message)
            out.header.frame_id = self.c['control']['world_frame']
            out.child_frame_id = 'base_link'
            out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = world_body[:3, 3]
            quat = quaternion_from_matrix(world_body)
            out.pose.pose.orientation.x, out.pose.pose.orientation.y, out.pose.pose.orientation.z, out.pose.pose.orientation.w = quat
            out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = velocity
            out.twist.twist.angular.x, out.twist.twist.angular.y, out.twist.twist.angular.z = angular
            self.odom_pub.publish(out)
        except (ValueError, TypeError) as exc:
            self.ros.logerr_throttle(1, str(exc))

    def camera(self):
        os.environ.setdefault('OPENCV_FFMPEG_CAPTURE_OPTIONS', 'rtsp_transport;tcp|stimeout;3000000')
        while not self.done.is_set():
            capture = cv2.VideoCapture(self.c['camera']['rtsp_url'], cv2.CAP_FFMPEG)
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            try:
                while not self.done.is_set() and capture.isOpened():
                    ok, frame = capture.read()
                    if not ok or frame is None:
                        break
                    if [frame.shape[1], frame.shape[0]] != self.c['hardware']['calibration']['image_size']:
                        self.ros.logerr_throttle(1, 'K40T image dimensions differ from calibration')
                        continue
                    received = self.ros.Time.now()
                    image = self.cv.cv2_to_imgmsg(frame, encoding='bgr8')
                    # Receipt time is not an exposure timestamp.
                    image.header.stamp = received
                    image.header.frame_id = self.c['hardware']['camera_optical_frame']
                    self.rgb_pub.publish(image)
            finally:
                capture.release()
            self.done.wait(.5)

    def close(self):
        self.done.set()
        self.worker.join(timeout=3.5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config')
    args = parser.parse_args()
    config = load_robot_config('i7', args.config)
    validate(config, require_geometry=False)
    import rospy
    rospy.init_node('i7_v22_sensors')
    Sensors(config)
    rospy.spin()


if __name__ == '__main__':
    main()
