"""SAM3 visual-reference detection, independent of the v21 src tree."""
import sys
from pathlib import Path

import numpy as np
from PIL import Image


class Sam3Detector:
    nms_iou_threshold = 0.8

    @staticmethod
    def box_iou(a, b):
        overlap = max(0., min(a[2], b[2]) - max(a[0], b[0])) * max(0., min(a[3], b[3]) - max(a[1], b[1]))
        union = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - overlap
        return overlap / union if union > 0 else 0.

    def __init__(self, model_root, checkpoint, device="cuda"):
        root, checkpoint = Path(model_root).resolve(), Path(checkpoint).resolve()
        if not root.is_dir() or not checkpoint.is_file():
            raise FileNotFoundError("SAM3 source directory or checkpoint not found")
        sys.path.insert(0, str(root))
        import torch
        from sam3.model.sam3_image_processor import Sam3Processor
        from sam3.model_builder import build_sam3_image_model

        if not device.startswith("cuda") or not torch.cuda.is_available():
            raise RuntimeError("SAM3 requires CUDA")
        with torch.cuda.device(device):
            if not torch.cuda.is_bf16_supported():
                raise RuntimeError("SAM3 requires BF16 support")
        model = build_sam3_image_model(
            device="cpu", checkpoint_path=str(checkpoint), load_from_HF=False,
            enable_segmentation=True, enable_inst_interactivity=False, compile=False,
        ).to(device=device).eval()
        self.processor = Sam3Processor(model, resolution=1008, device=device, confidence_threshold=0.5)
        self.torch, self.device = torch, device

    @staticmethod
    def compose(target, reference, box):
        # Same native-resolution vertical composition as v21: target above reference.
        th, tw = target.shape[:2]
        rh, rw = reference.shape[:2]
        width, height = max(tw, rw), th + rh
        tx, rx = (width - tw) // 2, (width - rw) // 2
        canvas = np.full((height, width, 3), 255, np.uint8)
        canvas[:th, tx:tx + tw] = target
        canvas[th:, rx:rx + rw] = reference
        x1, y1, x2, y2 = box
        prompt = [(rx + (x1 + x2) / 2) / width, (th + (y1 + y2) / 2) / height,
                  (x2 - x1) / width, (y2 - y1) / height]
        return canvas, prompt, (tx, 0, tx + tw, th)

    def detect(self, image, reference_image, box_xyxy):
        canvas, prompt, target_rect = self.compose(image, reference_image, box_xyxy)
        torch = self.torch
        state = None
        try:
            with torch.inference_mode(), torch.autocast("cuda", dtype=torch.bfloat16):
                state = self.processor.set_image(Image.fromarray(canvas))
                state = self.processor.add_geometric_prompt(box=prompt, label=True, state=state)
            torch.cuda.synchronize(self.device)
            return self.extract(state, target_rect)
        finally:
            del state
            with torch.cuda.device(self.device):
                torch.cuda.empty_cache()

    @staticmethod
    def extract(state, target_rect):
        boxes = state["boxes"].detach().float().cpu().numpy().reshape(-1, 4)
        scores = state["scores"].detach().float().cpu().numpy().reshape(-1)
        masks = state["masks"]
        if masks.ndim == 4 and masks.shape[1] == 1:
            masks = masks[:, 0]
        tx1, ty1, tx2, ty2 = target_rect
        if (len(boxes) != len(scores) or len(boxes) != len(masks) or masks.ndim != 3
                or masks.shape[-2] < ty2 or masks.shape[-1] < tx2):
            raise RuntimeError("Invalid SAM3 output dimensions")
        results = []
        for index in np.argsort(-scores, kind="stable"):
            box, score = boxes[index], float(scores[index])
            if not np.isfinite(box).all() or not np.isfinite(score) or not 0 <= score <= 1:
                raise RuntimeError("Invalid SAM3 box or confidence")
            if score <= 0.5:
                continue
            x1, y1, x2, y2 = box
            if not (tx1 <= (x1 + x2) / 2 < tx2 and ty1 <= (y1 + y2) / 2 < ty2):
                continue
            clipped = [float(np.clip(x1, tx1, tx2) - tx1), float(np.clip(y1, ty1, ty2) - ty1),
                       float(np.clip(x2, tx1, tx2) - tx1), float(np.clip(y2, ty1, ty2) - ty1)]
            if clipped[2] <= clipped[0] or clipped[3] <= clipped[1]:
                continue
            # Within-frame NMS before top3, matching v21's 0.80 box-IoU rule.
            if any(Sam3Detector.box_iou(clipped, item['box']) >= Sam3Detector.nms_iou_threshold
                   for item in results):
                continue
            # Copy only selected instance masks, cropped to target-frame pixels.
            mask = masks[int(index), ty1:ty2, tx1:tx2].detach().cpu().numpy()
            if mask.dtype != np.bool_:
                raise RuntimeError("SAM3 mask is not binary")
            if not mask.any():
                continue
            results.append({"box": clipped, "confidence": score, "mask": mask.copy()})
            if len(results) == 3:
                break
        return results
