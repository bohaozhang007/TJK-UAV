import os
from pathlib import Path
import sys

# Reserve the binary reply pipe before any third-party imports emit logs.
output = sys.stdout.buffer
sys.stdout = sys.stderr

import cv2
import numpy as np


from depth_base import DepthEstimator, run_depth


MODEL_ROOT = Path(__file__).resolve().parents[3] / "depth-anything-3"
sys.path.insert(0, str(MODEL_ROOT / "src"))
os.environ.setdefault("XFORMERS_FORCE_DISABLE_TRITON", "1")
from depth_anything_3.api import DepthAnything3

CHECKPOINT = MODEL_ROOT / "checkpoints/DA3NESTED-GIANT-LARGE"


class Da3Depth(DepthEstimator):
    def __init__(self):
        self.model = DepthAnything3.from_pretrained(str(CHECKPOINT)).to("cuda").eval()

    def estimate(self, img):
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        prediction = self.model.inference([rgb])
        if not prediction.is_metric:
            raise RuntimeError("DA3 did not return metric depth")
        depth = np.asarray(prediction.depth[0], dtype=np.float32)
        intrinsics = np.array(prediction.intrinsics[0], dtype=float)
        if (
            depth.ndim != 2
            or intrinsics.shape != (3, 3)
            or not np.isfinite(intrinsics).all()
            or intrinsics[0, 0] <= 0
            or intrinsics[1, 1] <= 0
        ):
            raise RuntimeError("DA3 returned invalid depth or intrinsics")

        # The default DA3 resize has no crop; scale depth and predicted K back together.
        h, w = img.shape[:2]
        intrinsics[0] *= w / depth.shape[1]
        intrinsics[1] *= h / depth.shape[0]
        depth = cv2.resize(
            depth,
            (w, h),
            interpolation=cv2.INTER_LINEAR,
        )
        return depth, intrinsics


run_depth(Da3Depth, "da3", output)
