"""YOLO-World-V2.1-X-640 adapter for agents and standalone inference.

Example:
    conda run -n yolo python yolo_world_x_640.py ^
        --image path/to/image.jpg --prompt "white light bulb"
"""

from __future__ import annotations

import argparse
import copy
import json
import sys
import time
from contextlib import nullcontext
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import cv2
import numpy as np
import torch


PROJECT_ROOT = Path(__file__).resolve().parents[3]
YOLO_WORLD_ROOT = PROJECT_ROOT / "YOLO-World"
DEFAULT_MODEL_CONFIG = (
    YOLO_WORLD_ROOT
    / "configs"
    / "pretrain"
    / (
        "yolo_world_v2_x_vlpan_bn_2e-3_100e_4x8gpus_"
        "obj365v1_goldg_cc3mlite_train_lvis_minival.py"
    )
)
DEFAULT_CHECKPOINT = (
    YOLO_WORLD_ROOT / "weights" / "yolo_world_v2.1_x_640.pth"
)
DEFAULT_IMAGE = YOLO_WORLD_ROOT / "demo" / "sample_images" / "bus.jpg"


def _prepare_imports() -> None:
    root = str(YOLO_WORLD_ROOT)
    if root not in sys.path:
        sys.path.insert(0, root)


_prepare_imports()

try:
    import yolo_world  # noqa: F401 - registers YOLO-World components
    from mmengine.config import Config
    from mmengine.dataset import Compose
    from mmdet.apis import init_detector
    from mmdet.utils import get_test_pipeline_cfg
except ModuleNotFoundError as exc:
    raise RuntimeError(
        "YOLO-World dependencies are unavailable. Activate the 'yolo' Conda "
        "environment before importing this module."
    ) from exc


@dataclass
class YOLOWorldX640Config:
    model_config: str = str(DEFAULT_MODEL_CONFIG)
    checkpoint: str = str(DEFAULT_CHECKPOINT)
    device: str = "cuda:0"
    input_size: int = 640
    score_threshold: float = 0.10
    nms_iou_threshold: float = 0.70
    max_detections: int = 100
    fp16: bool = True


def _validate_config(cfg: YOLOWorldX640Config) -> None:
    if not Path(cfg.model_config).expanduser().is_file():
        raise FileNotFoundError(f"YOLO-World config not found: {cfg.model_config}")
    if not Path(cfg.checkpoint).expanduser().is_file():
        raise FileNotFoundError(f"YOLO-World checkpoint not found: {cfg.checkpoint}")
    if cfg.input_size <= 0 or cfg.input_size % 32 != 0:
        raise ValueError("input_size must be a positive multiple of 32")
    if not 0.0 <= cfg.score_threshold <= 1.0:
        raise ValueError("score_threshold must be in [0, 1]")
    if not 0.0 <= cfg.nms_iou_threshold <= 1.0:
        raise ValueError("nms_iou_threshold must be in [0, 1]")
    if cfg.max_detections < 1:
        raise ValueError("max_detections must be positive")
    if cfg.fp16 and not cfg.device.startswith("cuda"):
        raise ValueError("fp16 inference requires a CUDA device")


def _patch_fp16_nms() -> None:
    """Keep the network in AMP while feeding FP32 boxes to Windows MMCV NMS."""
    from mmdet.models.dense_heads import base_dense_head

    if getattr(base_dense_head, "_yolo_world_fp16_nms_patched", False):
        return

    original_batched_nms = base_dense_head.batched_nms

    def batched_nms_fp32(
        boxes,
        scores,
        idxs,
        nms_cfg,
        class_agnostic=False,
    ):
        return original_batched_nms(
            boxes.float(),
            scores.float(),
            idxs,
            nms_cfg,
            class_agnostic=class_agnostic,
        )

    base_dense_head.batched_nms = batched_nms_fp32
    base_dense_head._yolo_world_fp16_nms_patched = True


def _set_nested_lazy_init(cfg: Config) -> None:
    dataset_cfg = cfg.test_dataloader.dataset
    if isinstance(dataset_cfg, dict) and "class_text_path" in dataset_cfg:
        class_text_path = Path(dataset_cfg["class_text_path"])
        if not class_text_path.is_absolute():
            dataset_cfg["class_text_path"] = str(
                YOLO_WORLD_ROOT / class_text_path
            )
    while isinstance(dataset_cfg, dict) and "dataset" in dataset_cfg:
        dataset_cfg = dataset_cfg["dataset"]
    if isinstance(dataset_cfg, dict):
        dataset_cfg["lazy_init"] = True
        if "data_root" in dataset_cfg:
            data_root = Path(dataset_cfg["data_root"])
            if not data_root.is_absolute():
                dataset_cfg["data_root"] = str(YOLO_WORLD_ROOT / data_root)


def _build_array_pipeline(cfg: Config, input_size: int) -> Compose:
    pipeline_cfg = copy.deepcopy(get_test_pipeline_cfg(cfg=cfg))
    array_pipeline = []
    for transform in pipeline_cfg:
        transform_type = str(transform.get("type", ""))
        short_type = transform_type.rsplit(".", 1)[-1]
        if short_type.startswith("LoadImage"):
            transform["type"] = "mmdet.LoadImageFromNDArray"
        elif short_type in {"LoadAnnotations", "LoadText"}:
            continue
        elif short_type == "PackDetInputs":
            meta_keys = transform.get("meta_keys", ())
            transform["meta_keys"] = tuple(
                key for key in meta_keys if key != "texts"
            )

        if "scale" in transform:
            transform["scale"] = (input_size, input_size)
        array_pipeline.append(transform)
    return Compose(array_pipeline)


def _normalize_prompts(prompt: str | Sequence[str]) -> tuple[str, ...]:
    raw_prompts = prompt.split(",") if isinstance(prompt, str) else prompt
    prompts = []
    seen = set()
    for item in raw_prompts:
        normalized = str(item).strip()
        if normalized and normalized not in seen:
            prompts.append(normalized)
            seen.add(normalized)
    if not prompts:
        raise ValueError("At least one non-empty detector prompt is required")
    return tuple(prompts)


class YOLOWorldX640Detector:
    """Agent-facing YOLO-World detector with persistent prompt embeddings."""

    def __init__(self, cfg: YOLOWorldX640Config):
        _validate_config(cfg)
        if cfg.device.startswith("cuda") and not torch.cuda.is_available():
            raise RuntimeError("CUDA was requested but is unavailable")

        self.cfg = cfg
        self.device = cfg.device
        self.fp16 = bool(cfg.fp16)
        self.prompt_labels: tuple[str, ...] | None = None
        self.prompt_texts: list[list[str]] | None = None
        self.prompt_cache_builds = 0
        self.last_timing = {
            "total_s": 0.0,
            "preprocess_s": 0.0,
            "inference_s": 0.0,
            "postprocess_s": 0.0,
        }

        model_cfg = Config.fromfile(str(Path(cfg.model_config).resolve()))
        _set_nested_lazy_init(model_cfg)
        model_cfg.model.test_cfg.score_thr = float(cfg.score_threshold)
        model_cfg.model.test_cfg.nms.iou_threshold = float(
            cfg.nms_iou_threshold
        )
        model_cfg.model.test_cfg.max_per_img = int(cfg.max_detections)

        if self.fp16:
            _patch_fp16_nms()

        self.model = init_detector(
            model_cfg,
            checkpoint=str(Path(cfg.checkpoint).resolve()),
            device=cfg.device,
        )
        self.model.eval()
        self.test_pipeline = _build_array_pipeline(model_cfg, cfg.input_size)

        precision = "fp16_amp" if self.fp16 else "fp32"
        print(
            f"[YOLO-INFO] Loaded YOLO-World-V2.1-X input={cfg.input_size} "
            f"device={cfg.device} precision={precision}"
        )

    def set_prompt(self, prompt: str | Sequence[str]) -> bool:
        """Cache text embeddings; return False when the same prompt is reused."""
        prompt_labels = _normalize_prompts(prompt)
        if prompt_labels == self.prompt_labels:
            return False

        # V2.1 expects one background/padding prompt. It is not exposed as a
        # user-visible detection label.
        prompt_texts = [list(prompt_labels) + [" "]]
        if self.device.startswith("cuda"):
            torch.cuda.synchronize()
        started = time.perf_counter()
        with torch.inference_mode():
            self.model.reparameterize(prompt_texts)
        if self.device.startswith("cuda"):
            torch.cuda.synchronize()

        self.prompt_labels = prompt_labels
        self.prompt_texts = prompt_texts
        self.prompt_cache_builds += 1
        print(
            f"[YOLO-INFO] Cached prompt embeddings build="
            f"{self.prompt_cache_builds} prompts={list(prompt_labels)!r} "
            f"time_s={time.perf_counter() - started:.4f}"
        )
        return True

    def detect(
        self,
        image_rgb: np.ndarray,
        prompt: str | Sequence[str] | None = None,
    ) -> list[dict]:
        """Return detections sorted by descending confidence."""
        total_started = time.perf_counter()
        if prompt is not None:
            self.set_prompt(prompt)
        if self.prompt_labels is None:
            raise RuntimeError("Call set_prompt() before detect()")

        image_rgb = np.asarray(image_rgb)
        if image_rgb.ndim != 3 or image_rgb.shape[2] != 3:
            raise ValueError("image_rgb must have shape (H, W, 3)")
        if image_rgb.dtype != np.uint8:
            image_rgb = np.clip(image_rgb, 0, 255).astype(np.uint8)

        preprocess_started = time.perf_counter()
        data_info = self.test_pipeline(
            {
                "img_id": 0,
                "img": np.ascontiguousarray(image_rgb),
            }
        )
        data_batch = {
            "inputs": data_info["inputs"].unsqueeze(0),
            "data_samples": [data_info["data_samples"]],
        }
        preprocess_s = time.perf_counter() - preprocess_started

        if self.device.startswith("cuda"):
            torch.cuda.synchronize()
        inference_started = time.perf_counter()
        autocast_context = (
            torch.amp.autocast("cuda", dtype=torch.float16)
            if self.fp16
            else nullcontext()
        )
        with torch.inference_mode(), autocast_context:
            output = self.model.test_step(data_batch)[0]
        if self.device.startswith("cuda"):
            torch.cuda.synchronize()
        inference_s = time.perf_counter() - inference_started

        postprocess_started = time.perf_counter()
        instances = output.pred_instances.cpu()
        boxes = instances.bboxes.numpy()
        scores = instances.scores.float().numpy()
        labels = instances.labels.numpy()
        detections = []
        for box, score, class_id in zip(boxes, scores, labels):
            class_id = int(class_id)
            if class_id >= len(self.prompt_labels):
                continue
            score = float(score)
            if score < self.cfg.score_threshold:
                continue
            detections.append(
                {
                    "box": np.asarray(box, dtype=np.float32),
                    "confidence": score,
                    "label": self.prompt_labels[class_id],
                    "class_id": class_id,
                }
            )
        detections.sort(key=lambda item: item["confidence"], reverse=True)
        detections = detections[: self.cfg.max_detections]
        postprocess_s = time.perf_counter() - postprocess_started
        self.last_timing = {
            "total_s": time.perf_counter() - total_started,
            "preprocess_s": preprocess_s,
            "inference_s": inference_s,
            "postprocess_s": postprocess_s,
        }
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


def _draw_detections(image_rgb: np.ndarray, detections: list[dict]) -> np.ndarray:
    image_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
    for detection in detections:
        x1, y1, x2, y2 = np.rint(detection["box"]).astype(int)
        label = f"{detection['label']} {detection['confidence']:.2f}"
        cv2.rectangle(image_bgr, (x1, y1), (x2, y2), (0, 0, 255), 2)
        cv2.putText(
            image_bgr,
            label,
            (x1, max(18, y1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
    return image_bgr


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Minimal YOLO-World-V2.1-X-640 detector example"
    )
    parser.add_argument("--image", default=str(DEFAULT_IMAGE))
    parser.add_argument("--prompt", default="bus,person")
    parser.add_argument("--output", default="./yolo_world_x_640_result.jpg")
    parser.add_argument("--model-config", default=str(DEFAULT_MODEL_CONFIG))
    parser.add_argument("--checkpoint", default=str(DEFAULT_CHECKPOINT))
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--input-size", type=int, default=640)
    parser.add_argument("--score-threshold", type=float, default=0.10)
    parser.add_argument("--nms-iou-threshold", type=float, default=0.70)
    parser.add_argument("--max-detections", type=int, default=100)
    parser.add_argument(
        "--fp16",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    return parser


def main() -> None:
    args = _build_arg_parser().parse_args()
    image_bgr = cv2.imread(args.image)
    if image_bgr is None:
        raise FileNotFoundError(f"Unable to read image: {args.image}")
    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)

    detector = YOLOWorldX640Detector(
        YOLOWorldX640Config(
            model_config=args.model_config,
            checkpoint=args.checkpoint,
            device=args.device,
            input_size=args.input_size,
            score_threshold=args.score_threshold,
            nms_iou_threshold=args.nms_iou_threshold,
            max_detections=args.max_detections,
            fp16=args.fp16,
        )
    )
    detector.set_prompt(args.prompt)
    detections = detector.detect(image_rgb)

    output_path = Path(args.output).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_path), _draw_detections(image_rgb, detections))
    serializable = [
        {
            **detection,
            "box": detection["box"].tolist(),
        }
        for detection in detections
    ]
    print(json.dumps(serializable, ensure_ascii=False, indent=2))
    print(
        f"[YOLO-INFO] detections={len(detections)} "
        f"timing={detector.last_timing} output={output_path}"
    )


if __name__ == "__main__":
    main()
