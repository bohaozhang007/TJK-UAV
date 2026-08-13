"""GroundingDINO adapter implementing the detector interface used by agents."""

from __future__ import annotations

import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np
from PIL import Image


PROJECT_ROOT = Path(__file__).resolve().parents[4]
GROUNDED_SAM_ROOT = PROJECT_ROOT / "sam2"
DEFAULT_MODEL_CONFIG = (
    GROUNDED_SAM_ROOT
    / "grounding_dino"
    / "groundingdino"
    / "config"
    / "GroundingDINO_SwinB_cfg.py"
)
DEFAULT_CHECKPOINT = (
    GROUNDED_SAM_ROOT
    / "gdino_checkpoints"
    / "groundingdino_swinb_cogcoor.pth"
)


@dataclass(frozen=True)
class GroundingDINOConfig:
    box_threshold: float = 0.35
    text_threshold: float = 0.25
    source_root: str = str(GROUNDED_SAM_ROOT)
    model_config: str = str(DEFAULT_MODEL_CONFIG)
    checkpoint: str = str(DEFAULT_CHECKPOINT)
    device: str = "cuda"


class GroundingDINODetector:
    """Lazy-loaded GroundingDINO backend with the common detector API."""

    def __init__(self, cfg: GroundingDINOConfig):
        source_root = Path(cfg.source_root).expanduser().resolve()
        package_root = source_root / "grounding_dino"
        if not package_root.is_dir():
            raise FileNotFoundError(
                f"GroundingDINO source directory not found: {package_root}"
            )
        for path_name, path_value in (
            ("model_config", cfg.model_config),
            ("checkpoint", cfg.checkpoint),
        ):
            if not Path(path_value).expanduser().is_file():
                raise FileNotFoundError(
                    f"GroundingDINO {path_name} not found: {path_value}"
                )
        if not 0.0 <= float(cfg.box_threshold) <= 1.0:
            raise ValueError("box_threshold must be in [0, 1]")
        if not 0.0 <= float(cfg.text_threshold) <= 1.0:
            raise ValueError("text_threshold must be in [0, 1]")

        sys.path.insert(0, os.fspath(package_root))
        import groundingdino.datasets.transforms as transforms
        from groundingdino.util.inference import load_model, predict

        self.cfg = cfg
        self._predict = predict
        self._prompt: str | None = None
        self.last_timing = {"total_s": 0.0, "inference_s": 0.0}
        self.model = load_model(
            str(Path(cfg.model_config).expanduser().resolve()),
            str(Path(cfg.checkpoint).expanduser().resolve()),
            device=cfg.device,
        )
        self.transform = transforms.Compose(
            [
                transforms.RandomResize([800], max_size=1333),
                transforms.ToTensor(),
                transforms.Normalize(
                    [0.485, 0.456, 0.406],
                    [0.229, 0.224, 0.225],
                ),
            ]
        )

    @staticmethod
    def _normalize_prompt(prompt: str | Sequence[str]) -> str:
        if isinstance(prompt, str):
            normalized = prompt.strip()
        else:
            normalized = ". ".join(
                str(item).strip() for item in prompt if str(item).strip()
            )
        if not normalized:
            raise ValueError("At least one non-empty detector prompt is required")
        return normalized

    def set_prompt(self, prompt: str | Sequence[str]) -> bool:
        normalized = self._normalize_prompt(prompt)
        if normalized == self._prompt:
            return False
        self._prompt = normalized
        return True

    def detect(
        self,
        image_rgb: np.ndarray,
        prompt: str | Sequence[str] | None = None,
    ) -> list[dict]:
        total_started = time.perf_counter()
        if prompt is not None:
            self.set_prompt(prompt)
        if self._prompt is None:
            raise RuntimeError("Call set_prompt() before detect()")

        image_tensor, _ = self.transform(Image.fromarray(image_rgb), None)
        inference_started = time.perf_counter()
        boxes, confidences, labels = self._predict(
            model=self.model,
            image=image_tensor,
            caption=self._prompt,
            box_threshold=float(self.cfg.box_threshold),
            text_threshold=float(self.cfg.text_threshold),
            device=self.cfg.device,
        )
        inference_s = time.perf_counter() - inference_started
        self.last_timing = {
            "total_s": time.perf_counter() - total_started,
            "inference_s": inference_s,
        }
        image_h, image_w = image_rgb.shape[:2]
        detections = []
        for box, confidence, label in zip(boxes, confidences, labels):
            cx, cy, box_w, box_h = box.detach().cpu().numpy()
            box_xyxy = np.array(
                [
                    (cx - box_w / 2) * image_w,
                    (cy - box_h / 2) * image_h,
                    (cx + box_w / 2) * image_w,
                    (cy + box_h / 2) * image_h,
                ],
                dtype=np.float32,
            )
            box_xyxy[[0, 2]] = np.clip(
                box_xyxy[[0, 2]],
                0,
                image_w - 1,
            )
            box_xyxy[[1, 3]] = np.clip(
                box_xyxy[[1, 3]],
                0,
                image_h - 1,
            )
            detections.append(
                {
                    "box": box_xyxy,
                    "confidence": float(confidence),
                    "label": str(label),
                }
            )
        detections.sort(
            key=lambda detection: detection["confidence"],
            reverse=True,
        )
        return detections

    def detect_best(
        self,
        image_rgb: np.ndarray,
        prompt: str | Sequence[str] | None = None,
    ):
        detections = self.detect(image_rgb, prompt=prompt)
        if not detections:
            return None
        best = detections[0]
        return best["box"], best["confidence"], best["label"]
