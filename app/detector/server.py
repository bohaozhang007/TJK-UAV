"""POST /detect: base64 image and optional frame_id; reference selected by --target.
Returns top-three boxes after NMS, confidence > 0.5 and original-image masks (row-major RLE).
"""
import argparse
import base64
import io
import json
import logging
import socket
import sys
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, HTTPServer
from logging.handlers import QueueHandler, QueueListener
from pathlib import Path
from queue import SimpleQueue

import numpy as np
from PIL import Image

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from app.detector.image_log import ImageLog

ASSETS_DIR = Path(__file__).resolve().parents[2] / "assets"


def start_logging(directory):
    events = SimpleQueue()
    handlers = (logging.StreamHandler(), logging.FileHandler(directory / "server.log", encoding="utf-8"))
    formatter = logging.Formatter("%(asctime)s %(levelname)s %(message)s")
    for handler in handlers:
        handler.setFormatter(formatter)
    logging.basicConfig(level=logging.INFO, format="%(message)s", handlers=[QueueHandler(events)], force=True)
    listener = QueueListener(events, *handlers)
    listener.start()
    return listener


def local_detector_ips():
    try:
        addresses = socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET)
        return ", ".join(sorted({entry[4][0] for entry in addresses
                                 if entry[4][0].startswith("192.168.")})) or "not found"
    except OSError:
        return "unavailable"


def build_detector(name, model_root=None, checkpoint=None, device="cuda"):
    if name == "sam3":
        from app.detector.third_party.sam3.engine import Sam3Detector
        root = model_root or Path(__file__).resolve().parents[3] / "sam3"
        return Sam3Detector(root, checkpoint or root / "sam3.pt", device)
    raise ValueError(f"Unknown detector: {name}")


def read_image(source):
    try:
        with Image.open(source) as image:
            if image.format not in {"JPEG", "PNG"} or image.width * image.height > 8_000_000:
                raise ValueError("image must be JPEG/PNG with at most 8,000,000 pixels")
            return np.array(image.convert("RGB"))
    except (OSError, Image.DecompressionBombError) as exc:
        raise ValueError("invalid image") from exc


def decode_image(value):
    if not isinstance(value, str) or not value:
        raise ValueError("image must be a base64 JPEG or PNG")
    return read_image(io.BytesIO(base64.b64decode(value, validate=True)))


def load_target(name):
    if not name or name in (".", "..") or any(c in name for c in "/\\:"):
        raise ValueError("target must be an asset name, not a path")
    images = [ASSETS_DIR / (name + suffix) for suffix in (".jpg", ".png")]
    existing = [path for path in images if path.is_file()]
    if len(existing) != 1:
        reason = "both exist" if existing else "neither exists"
        raise ValueError(f"Expected exactly one of {images[0]} or {images[1]}: {reason}")
    box_path = ASSETS_DIR / (name + ".txt")
    if not box_path.is_file():
        raise ValueError(f"Missing target box: {box_path}")
    reference = read_image(existing[0])
    try:
        box = [float(v) for v in box_path.read_text(encoding="utf-8-sig").split()]
    except ValueError as exc:
        raise ValueError(f"{box_path} must contain x1 y1 x2 y2") from exc
    if (len(box) != 4 or not np.isfinite(box).all()
            or not 0 <= box[0] < box[2] <= reference.shape[1]
            or not 0 <= box[1] < box[3] <= reference.shape[0]):
        raise ValueError(f"{box_path} must contain a valid x1 y1 x2 y2 box inside the reference image")
    return reference, box


def encode_mask(mask):
    # Alternating background/foreground lengths, starting with background.
    flat = mask.ravel(order="C")
    changes = np.flatnonzero(flat[1:] != flat[:-1]) + 1
    counts = np.diff(np.r_[0, changes, flat.size]).tolist()
    if flat[0]:
        counts.insert(0, 0)
    return {"size": list(mask.shape), "encoding": "rle", "order": "C", "counts": counts}


class DetectorHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        logging.info("HTTP client=%s %s", self.client_address[0], format % args)

    def setup(self):
        super().setup()
        self.connection.settimeout(15)

    def reply(self, status, payload):
        body = json.dumps(payload, allow_nan=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/health":
            self.reply(200, {"ok": True})
        else:
            self.reply(404, {"ok": False, "error": "unknown endpoint"})

    def do_POST(self):
        if self.path != "/detect":
            self.reply(404, {"ok": False, "error": "unknown endpoint"})
            return
        received = time.perf_counter()
        timings = {}
        logging.info("Request received client=%s content_length=%s", self.client_address[0],
                     self.headers.get("Content-Length"))
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if self.headers.get("Transfer-Encoding") or not 0 < length <= 32 * 1024 * 1024:
                raise ValueError("provide Content-Length between 1 and 32 MiB")
            data = json.loads(self.rfile.read(length))
            if not isinstance(data, dict):
                raise ValueError("request must be a JSON object")
            image = decode_image(data.get("image"))
            frame_id = data.get("frame_id")
            if frame_id is not None and not isinstance(frame_id, str):
                raise ValueError("frame_id must be a string")
        except (ValueError, OSError) as exc:
            logging.warning("Request rejected client=%s elapsed_s=%.3f error=%s",
                            self.client_address[0], time.perf_counter() - received, exc)
            self.reply(400, {"ok": False, "error": str(exc), "timings": {
                "receive_decode": {"status": "error", "elapsed_s": time.perf_counter()-received}}})
            return
        try:
            logging.info("Inference started frame=%r image=%sx%s receive_decode_s=%.3f", frame_id,
                         image.shape[1], image.shape[0], time.perf_counter() - received)
            started = time.perf_counter()
            timings['receive_decode'] = dict(status='ok', elapsed_s=started-received)
            try:
                detections = self.server.detector.detect(image, self.server.reference_image, self.server.reference_box)
            except Exception:
                timings['inference'] = dict(status='error', elapsed_s=time.perf_counter()-started)
                raise
            inferred = time.perf_counter()
            timings['inference'] = dict(status='ok', elapsed_s=inferred-started)
            logging.info("Inference finished frame=%r detections=%s inference_s=%.3f",
                         frame_id, len(detections), inferred - started)
            try:
                encoded = [{**item, "mask": encode_mask(item["mask"])} for item in detections]
            except Exception:
                timings['mask_encode'] = dict(status='error', elapsed_s=time.perf_counter()-inferred)
                raise
            timings['mask_encode'] = dict(status='ok', elapsed_s=time.perf_counter()-inferred)
            logging.info("Masks encoded frame=%r encode_s=%.3f", frame_id, time.perf_counter() - inferred)
            result = {"ok": True, "image_size": [image.shape[1], image.shape[0]],
                      "detections": encoded, "elapsed_s": time.perf_counter() - started, "timings": timings}
            if frame_id is not None:
                result["frame_id"] = frame_id
        except Exception as exc:
            logging.exception("Detection failed frame=%r elapsed_s=%.3f", frame_id, time.perf_counter() - received)
            self.reply(500, {"ok": False, "error": str(exc), "timings": timings})
            return
        try:
            sending = time.perf_counter()
            self.reply(200, result)
            logging.info("Response written frame=%r send_s=%.3f total_s=%.3f",
                         frame_id, time.perf_counter() - sending, time.perf_counter() - received)
        except OSError as exc:
            logging.warning("Response failed frame=%r total_s=%.3f client=%s error=%s",
                            frame_id, time.perf_counter() - received, self.client_address[0], exc)
        finally:
            self.server.image_log.submit(image, detections)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--det", choices=["sam3"], required=True)
    parser.add_argument("--target", required=True, help="asset name, e.g. moli3")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8790)
    parser.add_argument("--model-root", type=Path)
    parser.add_argument("--checkpoint", type=Path)
    parser.add_argument("--device", default="cuda")
    args = parser.parse_args()
    try:
        reference, box = load_target(args.target)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    # A single HTTP worker serializes model calls; no prompt/session lifecycle.
    with HTTPServer((args.host, args.port), DetectorHandler) as server:
        server.reference_image, server.reference_box = reference, box
        log_dir = (Path(__file__).resolve().parents[2] / "logs" / "detector"
                   / datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
        server.image_log = ImageLog(log_dir)
        listener = start_logging(log_dir)
        try:
            logging.info("Local IP (192.168.*.*): %s", local_detector_ips())
            logging.info("Detection images and server.log: %s", log_dir)
            logging.info("Loading detector=%s target=%s device=%s", args.det, args.target, args.device)
            started = time.perf_counter()
            server.detector = build_detector(args.det, args.model_root, args.checkpoint, args.device)
            logging.info("Detector loaded load_s=%.3f", time.perf_counter() - started)
            logging.info("Warmup started image=1280x960")
            started = time.perf_counter()
            server.detector.detect(np.zeros((960, 1280, 3), dtype=np.uint8), reference, box)
            logging.info("Warmup finished warmup_s=%.3f", time.perf_counter() - started)
            logging.info("Detector ready at http://%s:%s/detect",
                         args.host, args.port)
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        except Exception:
            logging.exception("Detector server failed")
            raise
        finally:
            server.image_log.close()
            listener.stop()
            for handler in listener.handlers:
                handler.close()


if __name__ == "__main__":
    main()
