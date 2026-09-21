"""POST /estimate {image: base64 PNG/JPEG, intrinsics: 3x3 K, output: depth|xyz}.
Input must be rectified, with K in input-image pixels. Returns float32 .npy in metres;
depth is camera Z, xyz axes are right/down/forward. Invalid values are NaN.
"""
import argparse
import base64
import io
import json
import logging
from pathlib import Path
import socket
import sys
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import numpy as np
from PIL import Image

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


def build_depth(model_root=None, checkpoint=None, device="cuda", process_res=504):
    from app.depth.third_party.da3.engine import Da3Depth
    root = model_root or Path(__file__).resolve().parents[3] / "depth-anything-3"
    return Da3Depth(root, checkpoint or root / "checkpoints/DA3NESTED-GIANT-LARGE", device, process_res)


def decode_request(data):
    if not isinstance(data, dict):
        raise ValueError("request must be a JSON object")
    value = data.get("image")
    if not isinstance(value, str) or not value:
        raise ValueError("image must be base64 PNG/JPEG")
    try:
        with Image.open(io.BytesIO(base64.b64decode(value, validate=True))) as image:
            if image.format not in {"PNG", "JPEG"} or image.width * image.height > 8_000_000:
                raise ValueError("image must be PNG/JPEG with at most 8,000,000 pixels")
            rgb = np.array(image.convert("RGB"))
    except (OSError, Image.DecompressionBombError) as exc:
        raise ValueError("invalid image") from exc
    try:
        k = np.asarray(data.get("intrinsics"), dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise ValueError("intrinsics must be a numeric 3x3 matrix") from exc
    if (k.shape != (3, 3) or not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0
            or not np.allclose(k[2], [0, 0, 1]) or not np.isclose(k[1, 0], 0)
            or not np.isfinite(k.astype(np.float32)).all()):
        raise ValueError("intrinsics must be a finite pinhole 3x3 K with positive fx/fy")
    output = data.get("output", "depth")
    if output not in ("depth", "xyz"):
        raise ValueError("output must be depth or xyz")
    return rgb, k, output


def depth_to_xyz(depth, intrinsics):
    v, u = np.indices(depth.shape, dtype=np.float32)
    rays = np.stack((u, v, np.ones_like(u)), axis=-1) @ np.linalg.inv(intrinsics).T
    return (rays * depth[..., None]).astype(np.float32)


class DepthHandler(BaseHTTPRequestHandler):
    def setup(self):
        super().setup()
        self.connection.settimeout(15)

    def log_message(self, format, *args):
        logging.info("client=%s %s", self.client_address[0], format % args)

    def reply(self, status, payload):
        self.send_body(status, json.dumps(payload, allow_nan=False).encode(), "application/json")

    def send_body(self, status, body, content_type, headers=None):
        try:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            for key, value in (headers or {}).items():
                self.send_header(key, str(value))
            self.end_headers()
            self.wfile.write(body)
        except OSError as exc:
            logging.warning("Response failed: %s", exc)

    def do_GET(self):
        if self.path == "/health":
            self.reply(200, {"ok": True, "model": "DA3NESTED-GIANT-LARGE", "unit": "m"})
        else:
            self.reply(404, {"ok": False, "error": "unknown endpoint"})

    def do_POST(self):
        if self.path != "/estimate":
            self.reply(404, {"ok": False, "error": "unknown endpoint"})
            return
        started = time.perf_counter()
        logging.info("Depth request received client=%s", self.client_address[0])
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if self.headers.get("Transfer-Encoding") or not 0 < length <= 32 * 1024 * 1024:
                raise ValueError("provide Content-Length between 1 and 32 MiB")
            image, k, output = decode_request(json.loads(self.rfile.read(length)))
        except (ValueError, OSError) as exc:
            self.reply(400, {"ok": False, "error": str(exc)})
            return
        try:
            depth = self.server.engine.estimate(image, k.astype(np.float32))
            array = depth if output == "depth" else depth_to_xyz(depth, k)
            buffer = io.BytesIO()
            np.save(buffer, np.asarray(array, dtype=np.float32), allow_pickle=False)
            elapsed = time.perf_counter() - started
        except Exception as exc:
            logging.exception("Depth estimation failed")
            self.reply(500, {"ok": False, "error": str(exc)})
            return
        self.send_body(200, buffer.getvalue(), "application/x-npy", {
            "X-Output": output, "X-Unit": "m", "X-Depth-Type": "camera-z",
            "X-Coordinate-Frame": "camera-optical-right-down-forward", "X-Elapsed-S": f"{elapsed:.3f}",
        })
        logging.info("Depth response output=%s shape=%s elapsed_s=%.3f", output, array.shape, elapsed)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8792)
    parser.add_argument("--model-root", type=Path)
    parser.add_argument("--checkpoint", type=Path)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--process-res", type=int, default=504)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    with HTTPServer((args.host, args.port), DepthHandler) as server:
        logging.info("Loading DA3NESTED-GIANT-LARGE")
        server.engine = build_depth(args.model_root, args.checkpoint, args.device, args.process_res)
        try:
            ips = sorted({v[4][0] for v in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET)
                          if v[4][0].startswith("192.168.")})
            logging.info("Local IP (192.168.*.*): %s", ", ".join(ips) or "not found")
        except OSError:
            logging.info("Local IP unavailable")
        try:
            logging.info("Warmup started image=1280x960")
            started = time.perf_counter()
            server.engine.warmup()
            logging.info("Warmup finished warmup_s=%.3f", time.perf_counter() - started)
            logging.info("Depth ready at http://%s:%s/estimate", args.host, args.port)
            server.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
