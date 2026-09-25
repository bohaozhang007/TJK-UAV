import pickle
import sys
from collections import OrderedDict
from pathlib import Path

# Reserve the binary reply pipe before any third-party imports emit logs.
output = sys.stdout.buffer
sys.stdout = sys.stderr

import cv2
import numpy as np
import torch
from PIL import Image


MODEL_ROOT = Path(__file__).resolve().parents[3] / "sam2"
sys.path.insert(0, str(MODEL_ROOT))
from sam2.build_sam import build_sam2_video_predictor

CHECKPOINT = MODEL_ROOT / "checkpoints/sam2.1_hiera_small.pt"
MODEL_CONFIG = "configs/sam2.1/sam2.1_hiera_s.yaml"
WARMUP_SHAPE = (1080, 1920, 3)


class Sam2Tracker:
    def __init__(self):
        if not torch.cuda.is_available() or not torch.cuda.is_bf16_supported():
            raise RuntimeError("SAM2 requires a CUDA GPU with bf16 support")
        self.predictor = build_sam2_video_predictor(MODEL_CONFIG, str(CHECKPOINT))
        self.device = self.predictor.device
        self.state = None
        self.frame_index = 0

    def image_tensor(self, img):
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        size = self.predictor.image_size
        pixels = np.array(Image.fromarray(rgb).resize((size, size)))
        tensor = torch.from_numpy(pixels).permute(
            2,
            0,
            1,
        ).to(self.device).float() / 255.0
        mean = tensor.new_tensor([0.485, 0.456, 0.406])[:, None, None]
        std = tensor.new_tensor([0.229, 0.224, 0.225])[:, None, None]
        return (tensor - mean) / std

    def reset(self):
        if self.state is not None:
            self.predictor.reset_state(self.state)
        self.state = None
        self.frame_index = 0

    def init(
        self,
        img,
        box,
    ):
        self.reset()
        box = np.asarray(box, dtype=np.float32)
        height, width = img.shape[:2]
        if (
            box.shape != (4,)
            or not np.isfinite(box).all()
            or not 0 <= box[0] < box[2] <= width
            or not 0 <= box[1] < box[3] <= height
        ):
            raise ValueError("Tracker box must be inside the current image")
        with torch.inference_mode():
            with torch.autocast("cuda", dtype=torch.bfloat16):
                # Match the local SAM2 video state without loading a video from disk.
                self.state = {
                    "images": {0: self.image_tensor(img)}, "num_frames": 1,
                    "video_height": height, "video_width": width,
                    "device": self.device, "storage_device": self.device,
                    "point_inputs_per_obj": {}, "mask_inputs_per_obj": {},
                    "cached_features": {}, "constants": {},
                    "obj_id_to_idx": OrderedDict(), "obj_idx_to_id": OrderedDict(),
                    "obj_ids": [], "output_dict_per_obj": {},
                    "temp_output_dict_per_obj": {}, "frames_tracked_per_obj": {},
                }
                _, _, masks = self.predictor.add_new_points_or_box(
                    self.state,
                    frame_idx=0,
                    obj_id=1,
                    box=box,
                )
                self.predictor.propagate_in_video_preflight(self.state)
                return mask_result(masks)

    def track(self, img):
        if self.state is None:
            raise RuntimeError("Initialize SAM2 before tracking")
        if img.shape[:2] != (self.state["video_height"], self.state["video_width"]):
            raise ValueError("Tracker image dimensions changed")
        with torch.inference_mode():
            with torch.autocast("cuda", dtype=torch.bfloat16):
                self.frame_index += 1
                index = self.frame_index
                self.state["images"] = {index: self.image_tensor(img)}
                self.state["num_frames"] = index + 1
                result = None
                for _, _, masks in self.predictor.propagate_in_video(
                    self.state,
                    start_frame_idx=index,
                    max_frame_num_to_track=0,
                ):
                    result = mask_result(masks)
                if result is None:
                    raise RuntimeError("SAM2 returned no tracking output")
                return result

    def warmup(self):
        img = np.zeros(WARMUP_SHAPE, dtype=np.uint8)
        self.init(img, [640, 360, 1280, 720])
        self.track(img)
        self.reset()


def mask_result(masks):
    mask = (masks[0, 0] > 0).detach().cpu().numpy()
    y, x = np.nonzero(mask)
    box = None if not len(x) else [int(x.min()), int(y.min()), int(x.max()) + 1, int(y.max()) + 1]
    return {"box": box, "mask": mask}


def main(output):
    try:
        pickle.load(sys.stdin.buffer)
        print("[sam2] Loading model...", flush=True)
        model = Sam2Tracker()
        print("[sam2] Model loaded.", flush=True)
        print("[sam2] Warming up at 1920 x 1080...", flush=True)
        model.warmup()
        print("[sam2] Warmup complete.", flush=True)
    except Exception as exc:
        pickle.dump((None, str(exc)), output)
        output.flush()
        return
    pickle.dump((None, None), output)
    output.flush()

    while True:
        try:
            img, box = pickle.load(sys.stdin.buffer)
        except EOFError:
            return
        try:
            result = model.track(img) if box is None else model.init(img, box)
        except Exception as exc:
            model.reset()
            pickle.dump((None, str(exc)), output)
            output.flush()
        else:
            pickle.dump((result, None), output)
            output.flush()


main(output)
