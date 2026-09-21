"""Metric single-image depth with calibrated focal scaling."""
import os
from pathlib import Path
import sys

import cv2
import numpy as np


class Da3Depth:
    def __init__(self, model_root, checkpoint, device="cuda", process_res=504):
        root, checkpoint = Path(model_root).resolve(), Path(checkpoint).resolve()
        if not (root / "src/depth_anything_3").is_dir() or not (checkpoint / "model.safetensors").is_file():
            raise FileNotFoundError("DA3 source or checkpoint not found")
        if process_res < 28:
            raise ValueError("process_res must be at least 28")
        sys.path.insert(0, str(root / "src"))
        os.environ.setdefault("XFORMERS_FORCE_DISABLE_TRITON", "1")
        from depth_anything_3.api import DepthAnything3

        self.model = DepthAnything3.from_pretrained(str(checkpoint)).to(device).eval()
        self.process_res = process_res
        # Nested scaling reads the main branch's output K, not its input K.
        self.model.model.da3.register_forward_hook(self._calibrated_intrinsics)

    @staticmethod
    def _calibrated_intrinsics(module, args, output):
        intrinsics = args[2]
        if intrinsics is None:
            raise ValueError("calibrated intrinsics are required")
        output.intrinsics = intrinsics
        return output

    def estimate(self, image, intrinsics):
        prediction = self.model.inference(
            [image], intrinsics=intrinsics[None], process_res=self.process_res,
            process_res_method="upper_bound_resize",
        )
        if not prediction.is_metric:
            raise RuntimeError("DA3 did not return metric depth")
        depth = np.asarray(prediction.depth[0], dtype=np.float32)
        if depth.ndim != 2:
            raise RuntimeError("invalid depth dimensions")
        # Single-image resize has no crop; recover the input pixel grid.
        if depth.shape != image.shape[:2]:
            depth = cv2.resize(depth, (image.shape[1], image.shape[0]), interpolation=cv2.INTER_LINEAR)
        valid = np.isfinite(depth) & (depth > 0)
        if not valid.any():
            raise RuntimeError("no valid depth pixels")
        depth[~valid] = np.nan
        return depth
