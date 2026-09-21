"""i7 sensor cache on the shared v22 bridge protocol."""
import time
import cv2
import numpy as np
from .owl_ego import OwlEgoHardware


class I7Hardware(OwlEgoHardware):
    snapshot_wait_s = .3

    def __init__(self, config):
        super().__init__(config)
        calibration = config['hardware']['calibration']
        self.rectify_size = tuple(calibration['image_size'])
        self.rectify_k = np.asarray(calibration['K'], float).reshape(3, 3)
        self.rectify_d = np.asarray(calibration['D'], float)
        # Build once before serving observations; preserve native K and size.
        self.rectify_maps = cv2.initUndistortRectifyMap(self.rectify_k, self.rectify_d,
            None, self.rectify_k, self.rectify_size, cv2.CV_16SC2)

    def rectify_rgb(self, bgr, k, d):
        if (bgr.shape[1::-1] != self.rectify_size or not np.array_equal(k, self.rectify_k)
                or not np.array_equal(d, self.rectify_d)):
            raise ValueError('i7 rectification geometry changed; restart Robot server')
        return cv2.remap(bgr, *self.rectify_maps, interpolation=cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_CONSTANT)

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
