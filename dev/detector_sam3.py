import argparse
import base64
import json
import sys
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

import cv2
import numpy as np


MODEL_ROOT = Path(__file__).resolve().parents[2] / "sam3"
CHECKPOINT = MODEL_ROOT / "sam3.pt"
SERVER_HOST = "0.0.0.0"
SERVER_PORT = 8790


def decode_img(encoded):
    data = np.frombuffer(base64.b64decode(encoded, validate=True), dtype=np.uint8)
    img = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError("Failed to decode image")
    return img


def load_reference(img_path, box_path):
    img = cv2.imdecode(np.fromfile(img_path, dtype=np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError("Failed to read reference image")
    box = np.array([float(v) for v in box_path.read_text(encoding="utf-8-sig").split()])
    h, w = img.shape[:2]
    if (box.shape != (4,) or not np.isfinite(box).all()
            or not 0 <= box[0] < box[2] <= w or not 0 <= box[1] < box[3] <= h):
        raise ValueError("Reference box file must contain x1 y1 x2 y2 within the image")
    return img, box


def box_iou(a, b):
    overlap = max(0, min(a[2], b[2]) - max(a[0], b[0])) * max(
        0, min(a[3], b[3]) - max(a[1], b[1]))
    union = ((a[2] - a[0]) * (a[3] - a[1])
             + (b[2] - b[0]) * (b[3] - b[1]) - overlap)
    return overlap / union if union > 0 else 0.0


def select_boxes(
    boxes,
    scores,
    target_rect,
    confidence_threshold,
    iou_threshold,
):
    """Sort by confidence, keep boxes in the target image, and apply IoU NMS."""
    left, top, right, bottom = target_rect
    selected = []
    for index in np.argsort(-scores, kind="stable"):
        x1, y1, x2, y2 = boxes[index]
        if scores[index] <= confidence_threshold:
            continue
        if not (left <= (x1 + x2) / 2 < right and top <= (y1 + y2) / 2 < bottom):
            continue
        box = [
            float(np.clip(
                x1,
                left,
                right,
            ) - left),
            float(np.clip(
                y1,
                top,
                bottom,
            ) - top),
            float(np.clip(
                x2,
                left,
                right,
            ) - left),
            float(np.clip(
                y2,
                top,
                bottom,
            ) - top),
        ]
        if box[2] <= box[0] or box[3] <= box[1]:
            continue
        if not any(box_iou(box, kept) >= iou_threshold for kept in selected):
            selected.append(box)
    return selected


class Sam3Detector:
    def __init__(self):
        sys.path.insert(0, str(MODEL_ROOT))
        import torch
        from sam3.model.sam3_image_processor import Sam3Processor
        from sam3.model_builder import build_sam3_image_model

        if not torch.cuda.is_available() or not torch.cuda.is_bf16_supported():
            raise RuntimeError("A CUDA GPU with BF16 support is required")
        model = build_sam3_image_model(checkpoint_path=str(CHECKPOINT))
        self.processor = Sam3Processor(model)
        self.torch = torch

    def detect(
        self,
        reference_img,
        reference_box,
        target_img,
        confidence_threshold=0.5,
        iou_threshold=0.7,
    ):
        from PIL import Image

        if not 0 <= confidence_threshold <= 1 or not 0 < iou_threshold <= 1:
            raise ValueError("confidence_threshold must be in [0, 1] and iou_threshold in (0, 1]")
        rh, rw = reference_img.shape[:2]
        th, tw = target_img.shape[:2]
        box = np.asarray(reference_box, dtype=float)
        if (box.shape != (4,) or not np.isfinite(box).all()
                or not 0 <= box[0] < box[2] <= rw or not 0 <= box[1] < box[3] <= rh):
            raise ValueError("reference_box must be [x1, y1, x2, y2] within the reference image")

        # Step 1: Resize the reference to the target width, preserving its aspect ratio.
        reference_height = max(1, round(rh * tw / rw))
        reference = cv2.resize(reference_img, (tw, reference_height))
        x1, y1, x2, y2 = box * [tw / rw, reference_height / rh,
                                tw / rw, reference_height / rh]

        # Step 2: Stack the target above the resized reference.
        stacked = np.vstack((target_img, reference))

        # Step 3: Pad the right or bottom with white to form a square.
        height, width = stacked.shape[:2]
        side = max(width, height)
        canvas = cv2.copyMakeBorder(
            stacked,
            0,
            side - height,
            0,
            side - width,
            cv2.BORDER_CONSTANT,
            value=(255, 255, 255),
        )

        prompt = [(x1 + x2) / (2 * side),
                  (th + (y1 + y2) / 2) / side,
                  (x2 - x1) / side, (y2 - y1) / side]

        self.processor.set_confidence_threshold(confidence_threshold)
        torch = self.torch
        with torch.inference_mode(), torch.autocast("cuda", dtype=torch.bfloat16):
            state = self.processor.set_image(Image.fromarray(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)))
            state = self.processor.add_geometric_prompt(
                box=prompt,
                label=True,
                state=state,
            )
        boxes = state["boxes"].detach().float().cpu().numpy().reshape(-1, 4)
        scores = state["scores"].detach().float().cpu().numpy().reshape(-1)
        return select_boxes(
            boxes,
            scores,
            (0, 0, tw, th),
            confidence_threshold,
            iou_threshold,
        )


class DetectorHandler(BaseHTTPRequestHandler):
    def reply(
        self,
        status,
        data,
    ):
        body = json.dumps(data, allow_nan=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        self.reply(200, {"ok": True}) if self.path == "/health" else self.reply(404, {"error": "not found"})

    def do_POST(self):
        if self.path != "/detect":
            self.reply(404, {"error": "not found"})
            return
        try:
            self.connection.settimeout(30)
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 64 * 1024 * 1024:
                raise ValueError("Request size must be between 1 byte and 64 MiB")
            data = json.loads(self.rfile.read(length))
            boxes = self.server.detector.detect(
                self.server.reference_img,
                self.server.reference_box,
                decode_img(data["current_img"]),
            )
        except Exception as exc:
            self.reply(500, {"error": str(exc)})
        else:
            self.reply(200, boxes)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--ref-img",
        type=Path,
        required=True,
        help="Local reference image path",
    )
    parser.add_argument(
        "--ref-box",
        type=Path,
        required=True,
        help="Text file containing pixel x1 y1 x2 y2",
    )
    args = parser.parse_args()
    try:
        reference_img, reference_box = load_reference(args.reference_img, args.reference_box)
    except (OSError, ValueError, cv2.error) as exc:
        parser.error(str(exc))

    # Load the model once and process inference requests sequentially.
    with HTTPServer((SERVER_HOST, SERVER_PORT), DetectorHandler) as server:
        server.reference_img, server.reference_box = reference_img, reference_box
        print("Loading SAM3...", flush=True)
        server.detector = Sam3Detector()
        print("Warming up...", flush=True)
        img = np.zeros((1080, 1920, 3), dtype=np.uint8)  # K40T resolution: 1920 x 1080.
        server.detector.detect(
            reference_img,
            reference_box,
            img,
        )
        print(f"Ready: http://{SERVER_HOST}:{SERVER_PORT}", flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
