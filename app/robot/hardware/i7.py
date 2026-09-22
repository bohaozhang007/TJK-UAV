"""i7 sensor cache on the shared v22 bridge protocol."""
import time
from collections import deque
from .owl_ego import OwlEgoHardware


class I7Hardware(OwlEgoHardware):
    snapshot_wait_s = .3
    rectify_observations = False

    def __init__(self, config):
        super().__init__(config)
        self.clouds = deque(maxlen=12)

    def start(self):
        super().start()
        from sensor_msgs.msg import PointCloud2
        self.handles.append(self.ros.Subscriber(self.c['topics']['cloud'], PointCloud2,
                            self._cloud, queue_size=1, buff_size=16777216))

    def _cloud(self, message):
        with self.lock:
            self.clouds.append((self.epoch, message))

    def exposure_cloud(self, stamp, epoch):
        with self.lock:
            messages = [m for e, m in self.clouds if e == epoch]
            return min(messages, key=lambda m: abs(m.header.stamp.to_sec()-stamp)) if messages else None

    def frame_after(self, deadline, timeout, guard, diagnostics=None):
        started = time.monotonic()
        limit = time.monotonic() + timeout
        while time.monotonic() < limit:
            guard()
            with self.camera_changed:
                image = self.image
                if (time.monotonic() >= deadline and image is not None
                        and image.header.stamp.to_sec() >= self.now_s()-(time.monotonic()-deadline)):
                    if diagnostics is not None:
                        diagnostics.update(frame_seq=int(image.header.seq),
                            frame_id=image.header.frame_id, stamp_s=image.header.stamp.to_sec(),
                            timestamp_source='sensor_receipt_not_exposure', exposure_stamp_s=None,
                            selected_monotonic_s=time.monotonic(), requested_after_monotonic_s=deadline,
                            receipt_age_s=self.now_s()-image.header.stamp.to_sec(),
                            wait_s=time.monotonic()-started)
                    return self.rgb_array(image).copy()
                self.camera_changed.wait(.05)
        raise RuntimeError('no newly received camera frame after settling')
