import pickle
import sys
from pathlib import Path

# Reserve the binary reply pipe before any third-party imports emit logs.
output = sys.stdout.buffer
sys.stdout = sys.stderr

import cv2
import numpy as np
import torch
from PIL import Image


MODEL_ROOT = Path(__file__).resolve().parents[3] / "sam3"
sys.path.insert(0, str(MODEL_ROOT))
from sam3.model.sam3_image_processor import Sam3Processor
from sam3.model_builder import build_sam3_image_model

CHECKPOINT = MODEL_ROOT / "sam3.pt"
WARMUP_SHAPE = (1080, 1920, 3)


class Sam3Detector:
    def __init__(
        self,
        reference_img,
        reference_box,
        target_shape,
    ):
        if (
            not torch.cuda.is_available()
            or not torch.cuda.is_bf16_supported()
        ):
            raise RuntimeError("A CUDA GPU with BF16 support is required")
        model = build_sam3_image_model(checkpoint_path=str(CHECKPOINT))
        self.processor = Sam3Processor(model)
        self.target_shape = tuple(target_shape)
        th, tw = self.target_shape
        rh, rw = reference_img.shape[:2]

        # Step 1: Resize the reference to the target width, preserving its aspect ratio.
        reference_height = max(1, round(rh * tw / rw))
        reference = cv2.resize(reference_img, (tw, reference_height))
        x1, y1, x2, y2 = np.asarray(reference_box) * [
            tw / rw, reference_height / rh, tw / rw, reference_height / rh,
        ]

        # Step 2: Reserve space for the target above the reference.
        height = th + reference_height

        # Step 3: Create the square with white right or bottom padding.
        side = max(tw, height)
        self.canvas = np.full(
            (side, side, 3),
            255,
            dtype=np.uint8,
        )
        self.canvas[th:height, :tw] = reference
        self.prompt = [(x1 + x2) / (2 * side),
                       (th + (y1 + y2) / 2) / side,
                       (x2 - x1) / side, (y2 - y1) / side]

    def detect(
        self,
        target_img,
        confidence_threshold=0.5,
        iou_threshold=0.7,
    ):
        if (
            not 0 <= confidence_threshold <= 1
            or not 0 < iou_threshold <= 1
        ):
            raise ValueError("confidence_threshold must be in [0, 1] and iou_threshold in (0, 1]")
        if target_img.shape != (*self.target_shape, 3):
            raise ValueError(f"Target image must have shape {(*self.target_shape, 3)}")
        th, tw = self.target_shape
        side = self.canvas.shape[0]
        self.canvas[:th, :tw] = target_img

        self.processor.set_confidence_threshold(confidence_threshold)
        with torch.inference_mode():
            with torch.autocast("cuda", dtype=torch.bfloat16):
                state = self.processor.set_image(Image.fromarray(cv2.cvtColor(self.canvas, cv2.COLOR_BGR2RGB)))
                state = self.processor.add_geometric_prompt(
                    box=self.prompt,
                    label=True,
                    state=state,
                )
        boxes = state["boxes"].detach().float().cpu().numpy().reshape(-1, 4)
        scores = state["scores"].detach().float().cpu().numpy().reshape(-1)
        detections = select_boxes(
            boxes,
            scores,
            (0, 0, tw, th),
            confidence_threshold,
            iou_threshold,
        )
        # Crop each selected SAM3 mask back to the unscaled current image.
        for item in detections:
            mask = state["masks"][item.pop("index")].reshape(side, side)[:th, :tw]
            item["mask"] = mask.detach().cpu().numpy().astype(bool)
        return detections

    def warmup(self):
        img = np.zeros(WARMUP_SHAPE, dtype=np.uint8)
        self.detect(img)


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
    """Apply target-region filtering and NMS while retaining source mask indices."""
    left, top, right, bottom = target_rect
    selected = []
    for index in np.argsort(-scores, kind="stable"):
        x1, y1, x2, y2 = boxes[index]
        if x2 <= x1 or y2 <= y1:
            continue
        if scores[index] <= confidence_threshold:
            break
        if not (
            left <= (x1 + x2) / 2 < right
            and top <= (y1 + y2) / 2 < bottom
        ):
            continue
        box = np.clip(
            boxes[index],
            [left, top, left, top],
            [right, bottom, right, bottom],
        )
        if not any(box_iou(box, kept["box"]) >= iou_threshold for kept in selected):
            selected.append({"box": box.tolist(), "confidence": float(scores[index]), "index": int(index)})
    return selected


def main(output):
    try:
        reference_img, reference_box, target_shape = pickle.load(sys.stdin.buffer)
        print("[sam3] Loading model...", flush=True)
        model = Sam3Detector(reference_img, reference_box, target_shape)
        print("[sam3] Model loaded.", flush=True)
        print("[sam3] Warming up at 1920 x 1080...", flush=True)
        model.warmup()
        print("[sam3] Warmup complete.", flush=True)
    except Exception as exc:
        pickle.dump((None, str(exc)), output)
        output.flush()
        return
    pickle.dump((None, None), output)
    output.flush()

    while True:
        try:
            img, = pickle.load(sys.stdin.buffer)
        except EOFError:
            return
        try:
            result = model.detect(img)
        except Exception as exc:
            pickle.dump((None, str(exc)), output)
            output.flush()
        else:
            pickle.dump((result, None), output)
            output.flush()


main(output)
