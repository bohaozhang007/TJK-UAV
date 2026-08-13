"""Agent-side client adapter for the persistent local SAM3 service."""

from __future__ import annotations

import base64
import json
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np


@dataclass(frozen=True)
class Sam3DetectorConfig:
    confidence_threshold: float = 0.5
    service_host: str = "127.0.0.1"
    service_port: int = 8780
    timeout_s: float = 180.0
    jpeg_quality: int = 95


class Sam3Detector:
    """Implement the common detector API without importing SAM3 locally."""

    SEARCH_COMPOSITE_LIMIT = 3

    def __init__(self, cfg: Sam3DetectorConfig) -> None:
        if not 0.0 <= float(cfg.confidence_threshold) <= 1.0:
            raise ValueError("confidence_threshold must be in [0, 1]")
        if not 1 <= int(cfg.jpeg_quality) <= 100:
            raise ValueError("jpeg_quality must be in [1, 100]")
        self.cfg = cfg
        self.base_url = f"http://{cfg.service_host}:{int(cfg.service_port)}"
        self._opener = urllib.request.build_opener(
            urllib.request.ProxyHandler({})
        )
        self._prompt_key = None
        self.prompt_cache_builds = 0
        self.last_timing = {"total_s": 0.0, "inference_s": 0.0}
        self.last_composite_path: str | None = None
        self._vis_dir: Path | None = None
        self._next_vis_name: str | None = None
        self._composite_index = 0
        self._saved_search_composites = 0
        self.health()

    def set_vis_dir(self, vis_dir: str) -> None:
        path = Path(vis_dir).expanduser().resolve()
        path.mkdir(parents=True, exist_ok=True)
        self._vis_dir = path
        self._next_vis_name = None
        self._composite_index = 0
        self._saved_search_composites = 0

    def set_next_vis_name(self, name: str) -> None:
        safe_name = "".join(
            char if char.isalnum() or char in {"-", "_"} else "_"
            for char in str(name)
        ).strip("_")
        if not safe_name:
            raise ValueError("SAM3 visualization name must not be empty")
        self._next_vis_name = safe_name

    def health(self) -> dict[str, Any]:
        request = urllib.request.Request(
            f"{self.base_url}/health",
            headers={"Accept": "application/json"},
            method="GET",
        )
        result = self._open_json(request, timeout_s=3.0)
        if result.get("ok") is not True or result.get("ready") is not True:
            raise RuntimeError(f"SAM3 service is not ready: {result}")
        if result.get("precision") != "bfloat16_amp":
            raise RuntimeError(
                "SAM3 service precision must be bfloat16_amp, got "
                f"{result.get('precision')!r}"
            )
        return result

    def set_prompt(self, prompt: str | dict[str, Any]) -> bool:
        if isinstance(prompt, str):
            text = prompt.strip()
            if not text:
                raise ValueError("text prompt must not be empty")
            prompt_key = ("text", text, float(self.cfg.confidence_threshold))
            payload = {
                "mode": "text",
                "text": text,
                "confidence_threshold": float(self.cfg.confidence_threshold),
            }
        elif isinstance(prompt, dict) and prompt.get("type") == "visual":
            image_path = Path(str(prompt.get("image_path", ""))).expanduser().resolve()
            if not image_path.is_file():
                raise FileNotFoundError(f"reference image not found: {image_path}")
            box = self._normalize_box(prompt.get("box_xyxy"))
            prompt_key = (
                "visual",
                str(image_path),
                image_path.stat().st_mtime_ns,
                box,
                float(self.cfg.confidence_threshold),
            )
            encoded = image_path.read_bytes()
            # Decode once here to reject unsupported/corrupt image formats. The
            # service receives a normalized JPEG independent of the source file.
            image_bgr = cv2.imdecode(
                np.frombuffer(encoded, dtype=np.uint8),
                cv2.IMREAD_COLOR,
            )
            if image_bgr is None:
                raise ValueError(f"failed to decode reference image: {image_path}")
            image_h, image_w = image_bgr.shape[:2]
            self._validate_box_bounds(box, image_w, image_h)
            payload = {
                "mode": "visual",
                "reference_image": base64.b64encode(encoded).decode("ascii"),
                "box_xyxy": list(box),
                "confidence_threshold": float(self.cfg.confidence_threshold),
            }
        else:
            raise TypeError(
                "SAM3 prompt must be text or a visual prompt mapping"
            )

        if prompt_key == self._prompt_key:
            return False
        result = self._post_json("/prompt", payload)
        if result.get("ok") is not True:
            raise RuntimeError(f"SAM3 prompt setup failed: {result}")
        self._prompt_key = prompt_key
        self.prompt_cache_builds += 1
        return True

    def detect(
        self,
        image_rgb: np.ndarray,
        prompt: str | dict[str, Any] | None = None,
        confidence_threshold: float | None = None,
    ) -> list[dict]:
        total_started = time.perf_counter()
        if prompt is not None:
            self.set_prompt(prompt)
        if self._prompt_key is None:
            raise RuntimeError("Call set_prompt() before detect()")
        image_rgb = np.asarray(image_rgb)
        if (
            image_rgb.ndim != 3
            or image_rgb.shape[2] != 3
            or image_rgb.dtype != np.uint8
        ):
            raise ValueError("SAM3 input must be a uint8 HxWx3 RGB array")
        image_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
        target_jpeg = self._encode_bgr(image_bgr)
        save_path = self._next_composite_path()
        payload = {
            "target_image": base64.b64encode(target_jpeg).decode("ascii"),
            "save_composite_path": (
                str(save_path) if save_path is not None else None
            ),
        }
        if confidence_threshold is not None:
            threshold = float(confidence_threshold)
            if not 0.0 <= threshold <= 1.0:
                raise ValueError("confidence_threshold must be in [0, 1]")
            payload["confidence_threshold"] = threshold
        result = self._post_json(
            "/detect",
            payload,
        )
        if result.get("ok") is not True:
            raise RuntimeError(f"SAM3 detection failed: {result}")
        timing = result.get("timing") or {}
        self.last_timing = {
            "total_s": time.perf_counter() - total_started,
            "inference_s": float(timing.get("inference_s", 0.0)),
        }
        compose = result.get("compose")
        self.last_composite_path = (
            str(compose.get("saved_path"))
            if isinstance(compose, dict) and compose.get("saved_path")
            else None
        )
        detections = result.get("detections")
        if not isinstance(detections, list):
            raise RuntimeError("SAM3 service returned invalid detections")
        normalized = []
        for detection in detections:
            if not isinstance(detection, dict):
                raise RuntimeError("SAM3 detection must be a JSON object")
            box = np.asarray(detection.get("box"), dtype=np.float32).reshape(-1)
            if box.size != 4 or not np.all(np.isfinite(box)):
                raise RuntimeError("SAM3 service returned an invalid box")
            normalized.append(
                {
                    "box": box,
                    "confidence": float(detection.get("confidence")),
                    "label": str(detection.get("label", "sam3")),
                }
            )
        normalized.sort(
            key=lambda detection: detection["confidence"],
            reverse=True,
        )
        return normalized

    def _next_composite_path(self) -> Path | None:
        try:
            if self._vis_dir is None or self._prompt_key is None:
                return None
            if self._prompt_key[0] != "visual":
                return None
            name = self._next_vis_name or f"detect_{self._composite_index:04d}"
            if name.startswith("search_view_"):
                if self._saved_search_composites >= self.SEARCH_COMPOSITE_LIMIT:
                    return None
                self._saved_search_composites += 1
            return self._vis_dir / f"sam3_input_{name}.png"
        finally:
            self._next_vis_name = None
            self._composite_index += 1

    def detect_best(self, image_rgb: np.ndarray, prompt=None):
        detections = self.detect(image_rgb, prompt=prompt)
        if not detections:
            return None
        best = detections[0]
        return best["box"], best["confidence"], best["label"]

    def _encode_bgr(self, image_bgr: np.ndarray) -> bytes:
        success, encoded = cv2.imencode(
            ".jpg",
            image_bgr,
            [cv2.IMWRITE_JPEG_QUALITY, int(self.cfg.jpeg_quality)],
        )
        if not success:
            raise RuntimeError("failed to encode image for SAM3 service")
        return encoded.tobytes()

    def _post_json(self, path: str, payload: dict[str, Any]) -> dict[str, Any]:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        request = urllib.request.Request(
            f"{self.base_url}{path}",
            data=body,
            headers={
                "Accept": "application/json",
                "Content-Type": "application/json; charset=utf-8",
            },
            method="POST",
        )
        return self._open_json(request)

    def _open_json(self, request, timeout_s: float | None = None) -> dict[str, Any]:
        timeout = self.cfg.timeout_s if timeout_s is None else float(timeout_s)
        try:
            with self._opener.open(request, timeout=timeout) as response:
                raw = response.read()
                content_type = response.headers.get_content_type()
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(
                f"HTTP {exc.code} from SAM3 service: {detail}"
            ) from exc
        except urllib.error.URLError as exc:
            raise RuntimeError(
                f"Cannot connect to SAM3 service at {self.base_url}: {exc}"
            ) from exc
        if content_type != "application/json":
            raise RuntimeError(
                f"Expected JSON from SAM3 service, got {content_type}"
            )
        try:
            result = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise RuntimeError("SAM3 service returned invalid JSON") from exc
        if not isinstance(result, dict):
            raise RuntimeError("SAM3 service did not return a JSON object")
        return result

    @staticmethod
    def _normalize_box(box_xyxy: Any) -> tuple[float, float, float, float]:
        try:
            values = tuple(float(value) for value in box_xyxy)
        except (TypeError, ValueError) as exc:
            raise ValueError("visual prompt box must contain four xyxy numbers") from exc
        if len(values) != 4 or not np.all(np.isfinite(values)):
            raise ValueError("visual prompt box must contain four finite xyxy numbers")
        return values

    @staticmethod
    def _validate_box_bounds(
        box: tuple[float, float, float, float],
        image_width: int,
        image_height: int,
    ) -> None:
        x1, y1, x2, y2 = box
        if not (0.0 <= x1 < x2 <= image_width):
            raise ValueError(
                f"box x coordinates must satisfy 0 <= x1 < x2 <= "
                f"{image_width}, got {box}"
            )
        if not (0.0 <= y1 < y2 <= image_height):
            raise ValueError(
                f"box y coordinates must satisfy 0 <= y1 < y2 <= "
                f"{image_height}, got {box}"
            )


__all__ = ["Sam3Detector", "Sam3DetectorConfig"]
