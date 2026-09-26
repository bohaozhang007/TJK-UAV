from pathlib import Path
import sys

# Reserve the binary reply pipe before any third-party imports emit logs.
output = sys.stdout.buffer
sys.stdout = sys.stderr

import cv2
import numpy as np
import torch


from depth_base import DepthEstimator, run_depth


MODEL_ROOT = Path(__file__).resolve().parents[3] / "MoGe"
sys.path.insert(0, str(MODEL_ROOT))
from moge.model.v3 import MoGeModel

CHECKPOINT = MODEL_ROOT / "ckpts/moge-3-vit-l/model.pt"


class Moge3Depth(DepthEstimator):
    def __init__(self):
        self.model = MoGeModel.from_pretrained(str(CHECKPOINT)).to("cuda").eval()

    def estimate(self, img):
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        image = torch.from_numpy(rgb).permute(2, 0, 1).to("cuda").float() / 255.0
        prediction = self.model.infer(image)
        depth = prediction["depth"].cpu().numpy().astype(np.float32, copy=True)
        intrinsics = prediction["intrinsics"].cpu().numpy().astype(float, copy=True)
        valid = prediction["mask"].cpu().numpy().astype(bool, copy=False)
        h, w = img.shape[:2]
        if (
            depth.shape != (h, w)
            or valid.shape != (h, w)
            or intrinsics.shape != (3, 3)
            or not np.isfinite(intrinsics).all()
            or intrinsics[0, 0] <= 0
            or intrinsics[1, 1] <= 0
        ):
            raise RuntimeError("MoGe-3 returned invalid depth or intrinsics")

        # MoGe returns metric Z at input resolution and normalized camera intrinsics.
        # Its UV pixel centers are (x + 0.5) / w, (y + 0.5) / h; mask_points
        # uses integer pixel centers, so subtract half a pixel after scaling K.
        intrinsics[0] *= w
        intrinsics[1] *= h
        intrinsics[:2, 2] -= 0.5
        depth[~valid] = np.nan
        return depth, intrinsics


run_depth(Moge3Depth, "moge3", output)
