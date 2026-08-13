"""Persistent local metric-depth service backed by Depth Anything 3."""

from __future__ import annotations

import argparse
import io
import json
import os
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, Optional
from urllib.parse import urlparse

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[4]
DEFAULT_DA3_ROOT = PROJECT_ROOT / "depth-anything-3"
DEFAULT_MODEL_DIR = (
    DEFAULT_DA3_ROOT
    / "checkpoints"
    / "DA3NESTED-GIANT-LARGE"
)
MAX_IMAGE_BYTES = 16 * 1024 * 1024


def _prepare_da3_import(da3_root: Path) -> None:
    package_root = da3_root / "src"
    package_dir = package_root / "depth_anything_3"
    if not package_dir.is_dir():
        raise RuntimeError(
            f"DA3 Python package not found: {package_dir}"
        )
    package_root_text = os.fspath(package_root)
    if package_root_text not in sys.path:
        sys.path.insert(0, package_root_text)


class Da3DepthEngine:
    """Load DA3 once and serialize GPU inference calls."""

    def __init__(
        self,
        da3_root: Path,
        model_dir: Path,
        device: str = "cuda",
        process_res: int = 504,
    ) -> None:
        self.da3_root = da3_root.expanduser().resolve()
        self.model_dir = model_dir.expanduser().resolve()
        self.device = str(device)
        self.process_res = int(process_res)
        self._lock = threading.Lock()
        self._model = None

    def load(self) -> None:
        if self.process_res <= 0:
            raise ValueError("process_res must be positive")
        weights_path = self.model_dir / "model.safetensors"
        if not weights_path.is_file():
            raise RuntimeError(f"DA3 weights not found: {weights_path}")

        _prepare_da3_import(self.da3_root)
        os.environ.setdefault("XFORMERS_FORCE_DISABLE_TRITON", "1")
        import torch
        from depth_anything_3.api import DepthAnything3

        if self.device.startswith("cuda") and not torch.cuda.is_available():
            raise RuntimeError("CUDA is required for DA3 depth estimation")
        self._model = DepthAnything3.from_pretrained(
            os.fspath(self.model_dir)
        ).to(self.device)

    def estimate_depth_cm(self, frame_rgb: np.ndarray) -> np.ndarray:
        if self._model is None:
            raise RuntimeError("DA3 model is not loaded")
        frame_rgb = np.asarray(frame_rgb)
        if (
            frame_rgb.ndim != 3
            or frame_rgb.shape[2] != 3
            or frame_rgb.dtype != np.uint8
        ):
            raise ValueError("DA3 input RGB must be a uint8 HxWx3 array")

        with self._lock:
            prediction = self._model.inference(
                [frame_rgb],
                process_res=self.process_res,
            )
        if not bool(prediction.is_metric):
            raise RuntimeError("DA3 did not return metric-scale depth")

        depth_m = np.asarray(prediction.depth[0], dtype=np.float32)
        if depth_m.ndim != 2:
            raise RuntimeError(
                f"DA3 returned invalid depth shape: {depth_m.shape}"
            )
        if depth_m.shape != frame_rgb.shape[:2]:
            depth_m = cv2.resize(
                depth_m,
                (frame_rgb.shape[1], frame_rgb.shape[0]),
                interpolation=cv2.INTER_LINEAR,
            )
        depth_cm = np.asarray(depth_m * 100.0, dtype=np.float32)
        valid = np.isfinite(depth_cm) & (depth_cm > 0)
        if not np.any(valid):
            raise RuntimeError("DA3 returned no valid positive metric depth")
        return depth_cm

    def health(self) -> Dict[str, Any]:
        return {
            "ok": self._model is not None,
            "ready": self._model is not None,
            "model_loaded": self._model is not None,
            "cuda": self.device.startswith("cuda"),
            "metric": True,
            "depth_unit": "cm",
            "model_dir": os.fspath(self.model_dir),
            "process_res": self.process_res,
        }


class Da3ApiHandler(BaseHTTPRequestHandler):
    engine: Optional[Da3DepthEngine] = None
    server_version = "TJK-DA3/1.0"

    def _json_response(self, code: int, payload: Dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _depth_response(self, depth_cm: np.ndarray) -> None:
        buffer = io.BytesIO()
        np.save(buffer, depth_cm, allow_pickle=False)
        body = buffer.getvalue()
        self.send_response(200)
        self.send_header("Content-Type", "application/x-npy")
        self.send_header("X-Depth-Unit", "cm")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        path = urlparse(self.path).path.rstrip("/") or "/"
        if path != "/health":
            self._json_response(
                404,
                {"ok": False, "error": f"unknown path: {path}"},
            )
            return
        if self.engine is None:
            self._json_response(
                503,
                {"ok": False, "ready": False, "error": "engine unavailable"},
            )
            return
        self._json_response(200, self.engine.health())

    def do_POST(self) -> None:
        path = urlparse(self.path).path.rstrip("/") or "/"
        if path != "/estimate":
            self._json_response(
                404,
                {"ok": False, "error": f"unknown path: {path}"},
            )
            return
        if self.engine is None:
            self._json_response(
                503,
                {"ok": False, "error": "engine unavailable"},
            )
            return

        try:
            content_length = int(self.headers.get("Content-Length", "0"))
            if content_length <= 0:
                raise ValueError("request image is empty")
            if content_length > MAX_IMAGE_BYTES:
                raise ValueError(
                    f"request image exceeds {MAX_IMAGE_BYTES} bytes"
                )
            encoded = self.rfile.read(content_length)
            frame_bgr = cv2.imdecode(
                np.frombuffer(encoded, dtype=np.uint8),
                cv2.IMREAD_COLOR,
            )
            if frame_bgr is None:
                raise ValueError("failed to decode request image")
            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            depth_cm = self.engine.estimate_depth_cm(frame_rgb)
            self._depth_response(depth_cm)
        except Exception as exc:
            self._json_response(400, {"ok": False, "error": str(exc)})

    def log_message(self, message_format: str, *args) -> None:
        print(
            f"[DA3-HTTP] {self.address_string()} "
            + message_format % args,
            flush=True,
        )


def run_server(
    engine: Da3DepthEngine,
    host: str = "127.0.0.1",
    port: int = 8770,
) -> None:
    Da3ApiHandler.engine = engine
    server = ThreadingHTTPServer((host, int(port)), Da3ApiHandler)
    print(
        f"[DA3-INFO] Ready at http://{host}:{int(port)} "
        f"model={engine.model_dir} unit=cm",
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
    parser.add_argument("--port", type=int, default=8770)
    parser.add_argument("--da3-root", type=Path, default=DEFAULT_DA3_ROOT)
    parser.add_argument("--model-dir", type=Path, default=DEFAULT_MODEL_DIR)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--process-res", type=int, default=504)
    return parser


def main() -> None:
    args = _build_arg_parser().parse_args()
    engine = Da3DepthEngine(
        da3_root=args.da3_root,
        model_dir=args.model_dir,
        device=args.device,
        process_res=args.process_res,
    )
    print(f"[DA3-INFO] Loading model from {engine.model_dir}", flush=True)
    engine.load()
    run_server(engine, host=args.host, port=args.port)


if __name__ == "__main__":
    main()
