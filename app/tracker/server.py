"""POST /init {image, box: [x1,y1,x2,y2]}; POST /track {image}. Image is base64 PNG/JPEG."""
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


def build_tracker(name, model_root=None, checkpoint=None, device="cuda"):
    if name == "sam2":
        from app.tracker.third_party.sam2.engine import Sam2Tracker
        root = model_root or Path(__file__).resolve().parents[3] / "sam2"
        return Sam2Tracker(root, checkpoint or root / "checkpoints/sam2.1_hiera_small.pt", device)
    raise ValueError(f"Unknown tracker: {name}")


def decode_image(value):
    if not isinstance(value, str) or not value:
        raise ValueError("image must be a base64 PNG or JPEG")
    try:
        with Image.open(io.BytesIO(base64.b64decode(value, validate=True))) as image:
            if image.format not in {"PNG", "JPEG"} or image.width * image.height > 8_000_000:
                raise ValueError("image must be PNG/JPEG with at most 8,000,000 pixels")
            return np.array(image.convert("RGB"))
    except (OSError, Image.DecompressionBombError) as exc:
        raise ValueError("invalid image") from exc


def validate_box(value, image):
    if (not isinstance(value, list) or len(value) != 4
            or any(type(v) not in (int, float) for v in value)
            or not np.isfinite(value).all()
            or not 0 <= value[0] < value[2] <= image.shape[1]
            or not 0 <= value[1] < value[3] <= image.shape[0]):
        raise ValueError("box must be [x1,y1,x2,y2] inside the image")
    return value


class TrackerHandler(BaseHTTPRequestHandler):
    def setup(self):
        super().setup()
        self.connection.settimeout(15)

    def log_message(self, format, *args):
        logging.info("client=%s %s", self.client_address[0], format % args)

    def reply(self, status, payload):
        body = json.dumps(payload, allow_nan=False).encode("utf-8")
        try:
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        except OSError as exc:
            logging.warning("Response failed: %s", exc)

    def do_GET(self):
        if self.path == "/health":
            self.reply(200, {"ok": True, "initialized": self.server.initialized})
        else:
            self.reply(404, {"ok": False, "error": "unknown endpoint"})

    def do_POST(self):
        if self.path not in ("/init", "/track"):
            self.reply(404, {"ok": False, "error": "unknown endpoint"})
            return
        started = time.perf_counter()
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if self.headers.get("Transfer-Encoding") or not 0 < length <= 32 * 1024 * 1024:
                raise ValueError("provide Content-Length between 1 and 32 MiB")
            data = json.loads(self.rfile.read(length))
            if not isinstance(data, dict):
                raise ValueError("request must be a JSON object")
            image = decode_image(data.get("image"))
            box = validate_box(data.get("box"), image) if self.path == "/init" else None
        except (ValueError, OSError) as exc:
            self.reply(400, {"ok": False, "error": str(exc)})
            return
        if self.path == "/track" and not self.server.initialized:
            self.reply(409, {"ok": False, "error": "call /init before /track"})
            return
        try:
            if self.path == "/init":
                self.server.initialized = False
                box = self.server.tracker.init(image, box)
                self.server.initialized = True
            else:
                box = self.server.tracker.track(image)
        except ValueError as exc:
            self.server.initialized = False
            self.server.tracker.reset()
            self.reply(400, {"ok": False, "error": str(exc)})
            return
        except Exception as exc:
            self.server.initialized = False
            self.server.tracker.reset()
            logging.exception("Tracking failed")
            self.reply(500, {"ok": False, "error": str(exc)})
            return
        elapsed = time.perf_counter() - started
        logging.info("%s image=%sx%s box=%s elapsed_s=%.3f", self.path, image.shape[1], image.shape[0], box, elapsed)
        self.reply(200, {"ok": True, "box": box, "found": box is not None,
                         "image_size": [image.shape[1], image.shape[0]], "elapsed_s": elapsed})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tracker", choices=["sam2"], default="sam2")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8791)
    parser.add_argument("--model-root", type=Path)
    parser.add_argument("--checkpoint", type=Path)
    parser.add_argument("--device", default="cuda")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    with HTTPServer((args.host, args.port), TrackerHandler) as server:
        server.initialized = False
        logging.info("Loading tracker=%s", args.tracker)
        server.tracker = build_tracker(args.tracker, args.model_root, args.checkpoint, args.device)
        try:
            addresses = sorted({v[4][0] for v in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET)
                                if v[4][0].startswith("192.168.")})
            logging.info("Local IP (192.168.*.*): %s", ", ".join(addresses) or "not found")
        except OSError:
            logging.info("Local IP unavailable")
        try:
            logging.info("Warmup started image=1280x960 (image encoder only)")
            started = time.perf_counter()
            server.tracker.warmup()
            logging.info("Warmup finished warmup_s=%.3f", time.perf_counter() - started)
            logging.info("Tracker ready at http://%s:%s", args.host, args.port)
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            server.tracker.reset()


if __name__ == "__main__":
    main()
