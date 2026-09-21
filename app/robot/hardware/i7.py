"""i7 sensor cache on the shared v22 bridge protocol."""
import time
from .owl_ego import OwlEgoHardware


class I7Hardware(OwlEgoHardware):
    snapshot_wait_s = .3
    rectify_observations = False

    def frame_after(self, deadline, timeout, guard):
        limit = time.monotonic() + timeout
        while time.monotonic() < limit:
            guard()
            with self.camera_changed:
                image = self.image
                if (time.monotonic() >= deadline and image is not None
                        and image.header.stamp.to_sec() >= self.now_s()-(time.monotonic()-deadline)
                        and 0 <= self.now_s()-image.header.stamp.to_sec() <= self.c['hardware']['rgb_max_age_s']):
                    return self.rgb_array(image).copy()
                self.camera_changed.wait(.05)
        raise RuntimeError('no newly received camera frame after settling')
