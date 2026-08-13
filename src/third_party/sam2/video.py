import os
import numpy as np
import torch
from PIL import Image

import sys
import cv2
from pathlib import Path

# ===== Add submodule path =====
WORKSPACE_ROOT = Path(__file__).resolve().parents[4]
SAM2_ROOT = WORKSPACE_ROOT / "sam2"
sys.path.insert(0, os.fspath(SAM2_ROOT))


from sam2.build_sam import build_sam2_video_predictor


def draw_box(img, box, color=(0,255,0), thickness=2):
    x1, y1, x2, y2 = map(int, box)
    img = img.copy()
    cv2.rectangle(img, (x1,y1), (x2,y2), color, thickness)
    return img


def draw_points(img, points, color=(0,0,255)):
    img = img.copy()
    for x, y in points.astype(int):
        cv2.circle(img, (x,y), 3, color, -1)
    return img


def overlay_mask(img, mask, color=(0,255,255), alpha=0.5):
    img = img.copy()
    m = mask.astype(bool)
    overlay = img.copy()
    overlay[m] = (overlay[m] * (1 - alpha) + np.array(color) * alpha).astype(np.uint8)
    return overlay


def save_visualization(
    save_dir,
    name,
    img,
    points=None,
    box=None,
    mask=None,
):
    os.makedirs(save_dir, exist_ok=True)

    # -------------------------
    # SRC visualization
    # -------------------------
    vis = img.copy()

    if box is not None:
        vis = draw_box(vis, box)

    if mask is not None:
        vis = overlay_mask(vis, mask)

    if points is not None:
        vis = draw_points(vis, points)

    out_path = os.path.join(save_dir, f"{name}.jpg")
    cv2.imwrite(out_path, cv2.cvtColor(vis, cv2.COLOR_RGB2BGR))
    print('[INFO] Saved in', out_path)

    return out_path


class Config:
    # Paths
    SAM2_CKPT = os.fspath(SAM2_ROOT / "checkpoints" / "sam2.1_hiera_small.pt")
    SAM2_CONFIG = "configs/sam2.1/sam2.1_hiera_s.yaml"

    # Device
    DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

    # Precision
    DTYPE = torch.bfloat16 if torch.cuda.is_available() and torch.cuda.get_device_capability()[0] >= 8 else torch.float16

    # Save figs
    SAVE_VIS = True
    VIS_DIR = "vis"

cfg = Config()

predictor = build_sam2_video_predictor(cfg.SAM2_CONFIG, cfg.SAM2_CKPT, device=cfg.DEVICE)


video_dir = os.fspath(SAM2_ROOT / "notebooks" / "videos" / "bedroom")

# scan all the JPEG frame names in this directory
frame_names = [
    p for p in os.listdir(video_dir)
    if os.path.splitext(p)[-1] in [".jpg", ".jpeg", ".JPG", ".JPEG"]
]
frame_names.sort(key=lambda p: int(os.path.splitext(p)[0]))
frame_idx = 0

inference_state = predictor.init_state(video_path=video_dir)

# predictor.reset_state(inference_state)

ann_frame_idx = 0 
ann_obj_id = 1  

points = np.array([[210, 350], [250, 220]], dtype=np.float32)
labels = np.array([1, 1], np.int32)
_, out_obj_ids, out_mask_logits = predictor.add_new_points_or_box(
    inference_state=inference_state,
    frame_idx=ann_frame_idx,
    obj_id=ann_obj_id,
    points=points,
    labels=labels,
)

print(np.array(Image.open(os.path.join(video_dir, frame_names[ann_frame_idx])).convert("RGB")).shape)
print((out_mask_logits[0, 0] > 0.0).cpu().numpy().shape)
print(points.shape)

save_visualization(
    cfg.VIS_DIR,
    f"sam2_0",
    np.array(Image.open(os.path.join(video_dir, frame_names[ann_frame_idx])).convert("RGB")),
    points,
    None,
    (out_mask_logits[0, 0] > 0.0).cpu().numpy(),
)

video_segments = {}  
for out_frame_idx, out_obj_ids, out_mask_logits in predictor.propagate_in_video(inference_state):
    video_segments[out_frame_idx] = {
        out_obj_id: (out_mask_logits[i] > 0.0).cpu().numpy()
        for i, out_obj_id in enumerate(out_obj_ids)
    }

vis_frame_stride = 30
for out_frame_idx in range(0, len(frame_names), vis_frame_stride):
    for out_obj_id, out_mask in video_segments[out_frame_idx].items():
        save_visualization(
            cfg.VIS_DIR,
            f"sam2_{out_frame_idx}_{out_obj_id}",
            np.array(Image.open(os.path.join(video_dir, frame_names[out_frame_idx])).convert("RGB")),
            None,
            None,
            out_mask[0],
        )
