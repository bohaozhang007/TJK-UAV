"""Persistent local SAM3 detector service for text and visual prompts."""

from __future__ import annotations

import argparse
import base64
import io
import json
import os
import sys
import threading
import time
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Optional
from urllib.parse import urlparse

import numpy as np
from PIL import Image


WORKSPACE_ROOT = Path(__file__).resolve().parents[4]
DEFAULT_SAM3_ROOT = WORKSPACE_ROOT / "sam3"
DEFAULT_CHECKPOINT = DEFAULT_SAM3_ROOT / "sam3.pt"
MAX_REQUEST_BYTES = 32 * 1024 * 1024


def _validate_rgb(image_rgb: np.ndarray, name: str) -> np.ndarray:
    image_rgb = np.asarray(image_rgb)
    if (
        image_rgb.ndim != 3
        or image_rgb.shape[2] != 3
        or image_rgb.dtype != np.uint8
    ):
        raise ValueError(f"{name} must be a uint8 HxWx3 RGB array")
    if image_rgb.shape[0] < 1 or image_rgb.shape[1] < 1:
        raise ValueError(f"{name} must not be empty")
    return image_rgb


def _decode_image(encoded: bytes, name: str) -> np.ndarray:
    try:
        with Image.open(io.BytesIO(encoded)) as image:
            return np.asarray(image.convert("RGB"), dtype=np.uint8)
    except Exception as exc:
        raise ValueError(f"failed to decode {name}") from exc


def _decode_base64_image(value: Any, name: str) -> np.ndarray:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{name} must be a non-empty base64 string")
    try:
        encoded = base64.b64decode(value, validate=True)
    except (ValueError, TypeError) as exc:
        raise ValueError(f"{name} is not valid base64") from exc
    return _decode_image(encoded, name)


def _validate_box_xyxy(
    box_xyxy: Any,
    image_width: int,
    image_height: int,
) -> np.ndarray:
    try:
        box = np.asarray(box_xyxy, dtype=np.float32).reshape(-1)
    except (TypeError, ValueError) as exc:
        raise ValueError("reference box must contain four xyxy numbers") from exc
    if box.size != 4 or not np.all(np.isfinite(box)):
        raise ValueError("reference box must contain four finite xyxy numbers")
    x1, y1, x2, y2 = (float(value) for value in box)
    if not (0.0 <= x1 < x2 <= float(image_width)):
        raise ValueError(
            f"reference box x coordinates must satisfy 0 <= x1 < x2 <= "
            f"{image_width}, got {box.tolist()}"
        )
    if not (0.0 <= y1 < y2 <= float(image_height)):
        raise ValueError(
            f"reference box y coordinates must satisfy 0 <= y1 < y2 <= "
            f"{image_height}, got {box.tolist()}"
        )
    return box


@dataclass(frozen=True)
class Sam3Composite:
    image_rgb: np.ndarray
    prompt_box_cxcywh_norm: tuple[float, float, float, float]
    target_rect_xyxy: tuple[int, int, int, int]
    reference_rect_xyxy: tuple[int, int, int, int]
    layout: str
    reference_scale: float


class Sam3VisualPromptComposer:
    """Place full target and reference images on a square SAM3 canvas."""

    def __init__(self, padding_color: int = 255) -> None:
        padding_color = int(padding_color)
        if not 0 <= padding_color <= 255:
            raise ValueError("padding_color must be in [0, 255]")
        self.padding_color = padding_color

    def compose(
        self,
        target_rgb: np.ndarray,
        reference_rgb: np.ndarray,
        reference_box_xyxy: Any,
    ) -> Sam3Composite:
        target_rgb = _validate_rgb(target_rgb, "target image")
        reference_rgb = _validate_rgb(reference_rgb, "reference image")
        target_h, target_w = target_rgb.shape[:2]
        reference_h, reference_w = reference_rgb.shape[:2]
        reference_box = _validate_box_xyxy(
            reference_box_xyxy,
            reference_w,
            reference_h,
        )

        target_short = min(target_h, target_w)
        reference_short = min(reference_h, reference_w)
        scale = float(target_short) / float(reference_short)
        resized_w = max(1, int(round(reference_w * scale)))
        resized_h = max(1, int(round(reference_h * scale)))
        resample = (
            Image.Resampling.LANCZOS
            if scale < 1.0
            else Image.Resampling.BILINEAR
        )
        resized_reference = np.asarray(
            Image.fromarray(reference_rgb).resize(
                (resized_w, resized_h),
                resample=resample,
            ),
            dtype=np.uint8,
        )

        vertical_side = max(max(target_w, resized_w), target_h + resized_h)
        horizontal_side = max(target_w + resized_w, max(target_h, resized_h))
        if vertical_side <= horizontal_side:
            layout = "bottom"
            canvas_side = vertical_side
            content_y = (canvas_side - target_h - resized_h) // 2
            target_x = (canvas_side - target_w) // 2
            target_y = content_y
            reference_x = (canvas_side - resized_w) // 2
            reference_y = content_y + target_h
        else:
            layout = "right"
            canvas_side = horizontal_side
            content_x = (canvas_side - target_w - resized_w) // 2
            target_x = content_x
            target_y = (canvas_side - target_h) // 2
            reference_x = content_x + target_w
            reference_y = (canvas_side - resized_h) // 2

        canvas = np.full(
            (canvas_side, canvas_side, 3),
            self.padding_color,
            dtype=np.uint8,
        )
        canvas[
            target_y : target_y + target_h,
            target_x : target_x + target_w,
        ] = target_rgb
        canvas[
            reference_y : reference_y + resized_h,
            reference_x : reference_x + resized_w,
        ] = resized_reference

        mapped = reference_box.copy()
        mapped[[0, 2]] *= float(resized_w) / float(reference_w)
        mapped[[1, 3]] *= float(resized_h) / float(reference_h)
        mapped[[0, 2]] += float(reference_x)
        mapped[[1, 3]] += float(reference_y)
        x1, y1, x2, y2 = (float(value) for value in mapped)
        prompt_box = (
            ((x1 + x2) * 0.5) / canvas_side,
            ((y1 + y2) * 0.5) / canvas_side,
            (x2 - x1) / canvas_side,
            (y2 - y1) / canvas_side,
        )
        return Sam3Composite(
            image_rgb=canvas,
            prompt_box_cxcywh_norm=prompt_box,
            target_rect_xyxy=(
                target_x,
                target_y,
                target_x + target_w,
                target_y + target_h,
            ),
            reference_rect_xyxy=(
                reference_x,
                reference_y,
                reference_x + resized_w,
                reference_y + resized_h,
            ),
            layout=layout,
            reference_scale=scale,
        )


class Sam3DetectorEngine:
    """Load SAM3 once and serialize prompt and inference operations."""

    def __init__(
        self,
        sam3_root: Path,
        checkpoint: Path,
        device: str = "cuda",
        confidence_threshold: float = 0.5,
        padding_color: int = 255,
    ) -> None:
        self.sam3_root = sam3_root.expanduser().resolve()
        self.checkpoint = checkpoint.expanduser().resolve()
        self.device = str(device)
        self.confidence_threshold = float(confidence_threshold)
        self.composer = Sam3VisualPromptComposer(padding_color)
        self._lock = threading.Lock()
        self._model = None
        self._processor = None
        self._torch = None
        self._prompt_mode: str | None = None
        self._text_prompt: str | None = None
        self._reference_rgb: np.ndarray | None = None
        self._reference_box: np.ndarray | None = None

    def load(self) -> None:
        if not self.sam3_root.is_dir():
            raise RuntimeError(f"SAM3 source directory not found: {self.sam3_root}")
        if not self.checkpoint.is_file():
            raise RuntimeError(f"SAM3 checkpoint not found: {self.checkpoint}")
        if not 0.0 <= self.confidence_threshold <= 1.0:
            raise ValueError("confidence_threshold must be in [0, 1]")

        root_text = os.fspath(self.sam3_root)
        if root_text not in sys.path:
            sys.path.insert(0, root_text)
        import torch
        from sam3.model.sam3_image_processor import Sam3Processor
        from sam3.model_builder import build_sam3_image_model

        if not self.device.startswith("cuda"):
            raise RuntimeError("SAM3 service currently requires a CUDA device")
        if not torch.cuda.is_available():
            raise RuntimeError("CUDA is required for SAM3 inference")
        if not torch.cuda.is_bf16_supported():
            raise RuntimeError("The selected CUDA device does not support BF16")

        # SAM3's official inference path uses BF16 autocast. Casting every
        # parameter to BF16 is not safe because a decoder checkpoint path can
        # still produce FP32 activations outside autocast-aware operators.
        model = build_sam3_image_model(
            device="cpu",
            checkpoint_path=os.fspath(self.checkpoint),
            load_from_HF=False,
            enable_segmentation=True,
            enable_inst_interactivity=False,
            compile=False,
        )
        model = model.to(device=self.device)
        model.eval()
        self._torch = torch
        self._model = model
        self._processor = Sam3Processor(
            model,
            resolution=1008,
            device=self.device,
            confidence_threshold=self.confidence_threshold,
        )

    def set_text_prompt(self, prompt: str, threshold: float) -> bool:
        normalized = str(prompt).strip()
        if not normalized:
            raise ValueError("text prompt must not be empty")
        threshold = self._validate_threshold(threshold)
        with self._lock:
            changed = (
                self._prompt_mode != "text"
                or self._text_prompt != normalized
                or self.confidence_threshold != threshold
            )
            self._prompt_mode = "text"
            self._text_prompt = normalized
            self._reference_rgb = None
            self._reference_box = None
            self._set_threshold(threshold)
        return changed

    def set_visual_prompt(
        self,
        reference_rgb: np.ndarray,
        reference_box_xyxy: Any,
        threshold: float,
    ) -> bool:
        reference_rgb = _validate_rgb(reference_rgb, "reference image")
        reference_h, reference_w = reference_rgb.shape[:2]
        reference_box = _validate_box_xyxy(
            reference_box_xyxy,
            reference_w,
            reference_h,
        )
        threshold = self._validate_threshold(threshold)
        with self._lock:
            self._prompt_mode = "visual"
            self._text_prompt = None
            self._reference_rgb = reference_rgb.copy()
            self._reference_box = reference_box.copy()
            self._set_threshold(threshold)
        return True

    def detect(
        self,
        target_rgb: np.ndarray,
        save_composite_path: str | None = None,
    ) -> dict[str, Any]:
        if self._processor is None or self._torch is None:
            raise RuntimeError("SAM3 model is not loaded")
        target_rgb = _validate_rgb(target_rgb, "target image")
        with self._lock:
            if self._prompt_mode is None:
                raise RuntimeError("set a text or visual prompt before detection")
            total_started = time.perf_counter()
            if self._prompt_mode == "text":
                model_input = target_rgb
                prompt_box = None
                target_rect = (0, 0, target_rgb.shape[1], target_rgb.shape[0])
                prompt_label = self._text_prompt or "text"
                compose_meta = None
            else:
                if self._reference_rgb is None or self._reference_box is None:
                    raise RuntimeError("visual prompt state is incomplete")
                composite = self.composer.compose(
                    target_rgb,
                    self._reference_rgb,
                    self._reference_box,
                )
                model_input = composite.image_rgb
                prompt_box = composite.prompt_box_cxcywh_norm
                target_rect = composite.target_rect_xyxy
                prompt_label = "visual_exemplar"
                compose_meta = {
                    "layout": composite.layout,
                    "canvas_shape": list(composite.image_rgb.shape),
                    "target_rect_xyxy": list(composite.target_rect_xyxy),
                    "reference_rect_xyxy": list(composite.reference_rect_xyxy),
                    "reference_scale": composite.reference_scale,
                    "prompt_box_cxcywh_norm": list(prompt_box),
                }
                if save_composite_path:
                    composite_path = self._save_composite(
                        model_input,
                        save_composite_path,
                    )
                    compose_meta["saved_path"] = composite_path

            inference_started = time.perf_counter()
            state = None
            try:
                with self._torch.inference_mode(), self._torch.autocast(
                    "cuda",
                    dtype=self._torch.bfloat16,
                ):
                    state = self._processor.set_image(Image.fromarray(model_input))
                    if self._prompt_mode == "text":
                        state = self._processor.set_text_prompt(
                            self._text_prompt,
                            state,
                        )
                    else:
                        state = self._processor.add_geometric_prompt(
                            box=list(prompt_box),
                            label=True,
                            state=state,
                        )
                if self.device.startswith("cuda"):
                    self._torch.cuda.synchronize()
                inference_s = time.perf_counter() - inference_started
                detections = self._state_to_detections(
                    state,
                    target_rect,
                    prompt_label,
                )
            finally:
                del state
                if self.device.startswith("cuda"):
                    self._torch.cuda.empty_cache()
            return {
                "ok": True,
                "detections": detections,
                "prompt_mode": self._prompt_mode,
                "compose": compose_meta,
                "timing": {
                    "total_s": time.perf_counter() - total_started,
                    "inference_s": inference_s,
                },
            }

    @staticmethod
    def _save_composite(
        composite_rgb: np.ndarray,
        save_path: str,
    ) -> str:
        path = Path(save_path).expanduser().resolve()
        if path.suffix.lower() != ".png":
            raise ValueError("SAM3 composite save path must end with .png")
        path.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(composite_rgb).save(path, format="PNG")
        return os.fspath(path)

    def _state_to_detections(
        self,
        state: dict,
        target_rect: tuple[int, int, int, int],
        label: str,
    ) -> list[dict[str, Any]]:
        boxes_tensor = state.get("boxes")
        scores_tensor = state.get("scores")
        if boxes_tensor is None or scores_tensor is None:
            return []
        boxes = boxes_tensor.detach().float().cpu().numpy().reshape(-1, 4)
        scores = scores_tensor.detach().float().cpu().numpy().reshape(-1)
        tx1, ty1, tx2, ty2 = target_rect
        detections = []
        for box, score in zip(boxes, scores):
            x1, y1, x2, y2 = (float(value) for value in box)
            center_x = (x1 + x2) * 0.5
            center_y = (y1 + y2) * 0.5
            if not (tx1 <= center_x < tx2 and ty1 <= center_y < ty2):
                continue
            clipped = [
                max(float(tx1), min(float(tx2), x1)) - tx1,
                max(float(ty1), min(float(ty2), y1)) - ty1,
                max(float(tx1), min(float(tx2), x2)) - tx1,
                max(float(ty1), min(float(ty2), y2)) - ty1,
            ]
            if clipped[2] <= clipped[0] or clipped[3] <= clipped[1]:
                continue
            detections.append(
                {
                    "box": clipped,
                    "confidence": float(score),
                    "label": label,
                }
            )
        detections.sort(
            key=lambda detection: detection["confidence"],
            reverse=True,
        )
        return detections

    def _validate_threshold(self, threshold: float) -> float:
        threshold = float(threshold)
        if not 0.0 <= threshold <= 1.0:
            raise ValueError("confidence_threshold must be in [0, 1]")
        return threshold

    def _set_threshold(self, threshold: float) -> None:
        self.confidence_threshold = threshold
        if self._processor is not None:
            self._processor.set_confidence_threshold(threshold)

    def health(self) -> dict[str, Any]:
        result = {
            "ok": self._model is not None,
            "ready": self._model is not None,
            "model_loaded": self._model is not None,
            "device": self.device,
            "precision": "bfloat16_amp",
            "resolution": 1008,
            "checkpoint": os.fspath(self.checkpoint),
            "prompt_mode": self._prompt_mode,
            "confidence_threshold": self.confidence_threshold,
        }
        if self._torch is not None and self.device.startswith("cuda"):
            result["cuda_memory_allocated_mib"] = round(
                self._torch.cuda.memory_allocated() / (1024**2),
                1,
            )
            result["cuda_memory_reserved_mib"] = round(
                self._torch.cuda.memory_reserved() / (1024**2),
                1,
            )
        return result


class Sam3ApiHandler(BaseHTTPRequestHandler):
    engine: Optional[Sam3DetectorEngine] = None
    server_version = "TJK-SAM3/1.0"

    def _json_response(self, code: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_body(self) -> bytes:
        content_length = int(self.headers.get("Content-Length", "0"))
        if content_length <= 0:
            raise ValueError("request body is empty")
        if content_length > MAX_REQUEST_BYTES:
            raise ValueError(f"request exceeds {MAX_REQUEST_BYTES} bytes")
        return self.rfile.read(content_length)

    def _read_json(self) -> dict[str, Any]:
        try:
            payload = json.loads(self._read_body().decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ValueError("request body must be a JSON object") from exc
        if not isinstance(payload, dict):
            raise ValueError("request body must be a JSON object")
        return payload

    def do_GET(self) -> None:
        path = urlparse(self.path).path.rstrip("/") or "/"
        if path != "/health":
            self._json_response(404, {"ok": False, "error": f"unknown path: {path}"})
            return
        if self.engine is None:
            self._json_response(503, {"ok": False, "ready": False})
            return
        self._json_response(200, self.engine.health())

    def do_POST(self) -> None:
        path = urlparse(self.path).path.rstrip("/") or "/"
        if self.engine is None:
            self._json_response(503, {"ok": False, "error": "engine unavailable"})
            return
        try:
            if path == "/prompt":
                payload = self._read_json()
                mode = payload.get("mode")
                threshold = payload.get(
                    "confidence_threshold",
                    self.engine.confidence_threshold,
                )
                if mode == "text":
                    changed = self.engine.set_text_prompt(
                        payload.get("text", ""),
                        threshold,
                    )
                elif mode == "visual":
                    reference_rgb = _decode_base64_image(
                        payload.get("reference_image"),
                        "reference image",
                    )
                    changed = self.engine.set_visual_prompt(
                        reference_rgb,
                        payload.get("box_xyxy"),
                        threshold,
                    )
                else:
                    raise ValueError("prompt mode must be 'text' or 'visual'")
                self._json_response(
                    200,
                    {
                        "ok": True,
                        "changed": changed,
                        "prompt_mode": mode,
                    },
                )
                return
            if path == "/detect":
                payload = self._read_json()
                target_rgb = _decode_base64_image(
                    payload.get("target_image"),
                    "target image",
                )
                save_path = payload.get("save_composite_path")
                if save_path is not None and not isinstance(save_path, str):
                    raise ValueError("save_composite_path must be a string")
                self._json_response(
                    200,
                    self.engine.detect(
                        target_rgb,
                        save_composite_path=save_path,
                    ),
                )
                return
            self._json_response(404, {"ok": False, "error": f"unknown path: {path}"})
        except Exception as exc:
            self._json_response(400, {"ok": False, "error": str(exc)})

    def log_message(self, message_format: str, *args) -> None:
        print(
            f"[SAM3-HTTP] {self.address_string()} " + message_format % args,
            flush=True,
        )


def run_server(
    engine: Sam3DetectorEngine,
    host: str = "127.0.0.1",
    port: int = 8780,
) -> None:
    Sam3ApiHandler.engine = engine
    server = ThreadingHTTPServer((host, int(port)), Sam3ApiHandler)
    print(
        f"[SAM3-INFO] Ready at http://{host}:{int(port)} "
        f"checkpoint={engine.checkpoint} precision=bfloat16_amp",
        flush=True,
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8780)
    parser.add_argument("--sam3-root", type=Path, default=DEFAULT_SAM3_ROOT)
    parser.add_argument("--checkpoint", type=Path, default=DEFAULT_CHECKPOINT)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--confidence-threshold", type=float, default=0.5)
    parser.add_argument("--padding-color", type=int, default=255)
    return parser


def main() -> None:
    args = _build_arg_parser().parse_args()
    engine = Sam3DetectorEngine(
        sam3_root=args.sam3_root,
        checkpoint=args.checkpoint,
        device=args.device,
        confidence_threshold=args.confidence_threshold,
        padding_color=args.padding_color,
    )
    print(f"[SAM3-INFO] Loading model from {engine.checkpoint}", flush=True)
    engine.load()
    run_server(engine, host=args.host, port=args.port)


if __name__ == "__main__":
    main()
