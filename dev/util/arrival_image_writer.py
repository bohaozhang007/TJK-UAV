from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from pathlib import Path

import rospy

from hardware.k40t import encode_img


TIMESTAMP_FORMAT = "%Y%m%d_%H%M%S_%f"


class ArrivalImageWriter:
    def __init__(self, run_dir):
        self.run_dir = Path(run_dir)
        self.worker = ThreadPoolExecutor(max_workers=1)

    def save(
        self,
        target_id,
        img,
    ):
        timestamp = datetime.now().strftime(TIMESTAMP_FORMAT)
        path = self.run_dir / f"{target_id}_{timestamp}.jpg"
        self.worker.submit(
            self.write,
            img,
            path,
        )

    def write(
        self,
        img,
        path,
    ):
        try:
            path.write_bytes(encode_img(img))
        except Exception as exc:
            rospy.logerr(f"Target image save failed: {path}: {exc}")
        else:
            rospy.loginfo(f"Target image saved: {path}")

    def close(self):
        self.worker.shutdown()
