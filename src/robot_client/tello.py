from __future__ import annotations

import os
import threading
from pathlib import Path
from typing import Tuple

import cv2
import numpy as np

from .base import BaseClient


class TelloClient(BaseClient):
    """Tello Robot Client with client-side DA3 depth estimation."""

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 8765,
        timeout_s: float = 180.0,
        *,
        depth_model_dir: str | Path | None = None,
    ) -> None:
        super().__init__(
            host=host,
            port=port,
            timeout_s=timeout_s,
        )
        self._depth_lock = threading.Lock()
        self._depth_model = None
        self._depth_model_dir = (
            Path(depth_model_dir)
            if depth_model_dir is not None
            else Path(__file__).resolve().parents[3]
            / "depth-anything-3"
            / "checkpoints"
            / "DA3NESTED-GIANT-LARGE"
        )

    def capture(
        self,
        include_depth: bool = True,
        raw: bool = True,
    ) -> np.ndarray | Tuple[np.ndarray, np.ndarray]:
        """Return 640x480 RGB and, optionally, DA3 metric depth in cm."""
        frame_rgb = super().capture(include_depth=False, raw=raw)
        if not isinstance(frame_rgb, np.ndarray):
            raise RuntimeError("BaseClient.capture did not return an RGB array")

        frame_rgb = cv2.resize(
            frame_rgb,
            (640, 480),
            interpolation=cv2.INTER_AREA,
        )
        if not include_depth:
            return frame_rgb
        return frame_rgb, self._estimate_depth_cm(frame_rgb)

    def _get_depth_model(self):
        with self._depth_lock:
            if self._depth_model is None:
                weights_path = self._depth_model_dir / "model.safetensors"
                if not weights_path.is_file():
                    raise RuntimeError(f"DA3 weights not found: {weights_path}")

                os.environ.setdefault("XFORMERS_FORCE_DISABLE_TRITON", "1")
                import torch
                from depth_anything_3.api import DepthAnything3

                if not torch.cuda.is_available():
                    raise RuntimeError("CUDA is required for DA3 depth estimation")
                self._depth_model = DepthAnything3.from_pretrained(
                    str(self._depth_model_dir)
                ).to("cuda")
            return self._depth_model

    def _estimate_depth_cm(self, frame_rgb: np.ndarray) -> np.ndarray:
        model = self._get_depth_model()
        with self._depth_lock:
            prediction = model.inference([frame_rgb], process_res=504)

        if not bool(prediction.is_metric):
            raise RuntimeError("DA3 did not return metric-scale depth")
        depth_m = np.asarray(prediction.depth[0], dtype=np.float32)
        depth_m = cv2.resize(
            depth_m,
            (frame_rgb.shape[1], frame_rgb.shape[0]),
            interpolation=cv2.INTER_LINEAR,
        )
        return depth_m * 100.0
