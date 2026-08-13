import os
import sys
import time
import shutil
from pathlib import Path

import hydra
import matplotlib
matplotlib.use('Agg')  # Don't show img in the call
import matplotlib.pyplot as plt
import numpy as np
import torch
from hydra.core.global_hydra import GlobalHydra
from PIL import Image
from tqdm import tqdm

WORKSPACE_ROOT = Path(__file__).resolve().parents[4]
SAM2_ROOT = WORKSPACE_ROOT / "sam2"
sys.path.insert(0, os.fspath(SAM2_ROOT))

from third_party.sam2.one_obj_video_predictor import build_one_obj_video_predictor
from utils import show_fig

DEFAULT_NAV_RUNS_ROOT = WORKSPACE_ROOT / "TJK-UAV" / "logs"


def mask2bbox(mask):
    ys, xs = np.where(mask > 0)
    if xs.size == 0 or ys.size == 0:
        return np.array([[0, 0], [0, 0]], dtype=np.int32)
    x1, x2 = xs.min(), xs.max()
    y1, y2 = ys.min(), ys.max()
    bbox = np.array([[x1, y1], [x2, y2]], dtype=np.int32)
    return bbox


def to_numpy_mask(mask):
    if hasattr(mask, "detach"):
        mask = mask.detach().cpu().numpy()
    arr = np.asarray(mask)
    arr = np.squeeze(arr)
    return (arr > 0).astype(np.uint8)

class SAM2Config:
    SAM2_CFG = 'sam2.1/sam2.1_hiera_s.yaml'
    SAM2_CKPT = os.fspath(SAM2_ROOT / "checkpoints" / "sam2.1_hiera_small.pt")
    VIDEO_FOLDER = os.fspath(SAM2_ROOT / "notebooks" / "videos" / "bedroom")
    HYDRA_CONFIG = os.fspath(SAM2_ROOT / "sam2" / "configs")
    LOCAL_RANK = 0
    DEVICE = "cuda"
    DTYPE = torch.bfloat16

    VIDEO_HEIGHT = 540
    VIDEO_WIDTH = 960

    SAVE_VIS = True
    VIS_DIR = os.fspath(DEFAULT_NAV_RUNS_ROOT / "sam2_stream_standalone")


class Sam2VideoPredictor:
    def __init__(self, cfg: SAM2Config):
        self.cfg = cfg
        self.device = cfg.DEVICE
        self.dtype = cfg.DTYPE
        os.environ["PYTORCH_CUDA_ALLOC_CONF"] = "expandable_segments:True"
        print(f"[SAM-INFO] SAM Using device: {self.device}")

        GlobalHydra.instance().clear()
        hydra.initialize_config_dir(config_dir=cfg.HYDRA_CONFIG, version_base=None)

        self.predictor = build_one_obj_video_predictor(
            cfg.SAM2_CFG,
            cfg.SAM2_CKPT,
            device=self.device
        )
        print("[SAM-INFO] Finish loading SAM")

        self.predictor.init_inference_state()
        self.predictor.video_height = cfg.VIDEO_HEIGHT
        self.predictor.video_width = cfg.VIDEO_WIDTH

        self.vis_dir = cfg.VIS_DIR
        self.save_vis = cfg.SAVE_VIS
        self.frame_idx = 0
        self.track_vis_prefix = "sam2_trak"
        self.track_vis_index_width = 0
        self.last_timing = {
            "inference_s": 0.0,
            "postprocess_s": 0.0,
            "visualization_s": 0.0,
            "total_s": 0.0,
        }


    def set_vis_dir(self, vis_dir):
        self.vis_dir = vis_dir

    def set_vis_mode(self, mode):
        self.save_vis = mode

    def set_track_vis_naming(self, prefix, index_width=2):
        prefix = str(prefix).strip()
        if not prefix:
            raise ValueError("Track visualization prefix cannot be empty")
        index_width = int(index_width)
        if index_width < 1:
            raise ValueError("Track visualization index width must be positive")
        self.track_vis_prefix = prefix
        self.track_vis_index_width = index_width
        
    def set_img_size(self, img_H, img_W):
        self.predictor.video_height = img_H
        self.predictor.video_width = img_W


    def forward(
        self,
        img_np,
        frame_idx,
        points=None,
        labels=None,
        box=None,
    ):
        if box is not None:
            if points is not None or labels is not None:
                raise ValueError("box cannot be combined with point prompts")
            points = np.asarray(box, dtype=np.float32).reshape(2, 2)
            labels = np.array([2, 3], dtype=np.int32)

        with torch.inference_mode():
            with torch.amp.autocast("cuda", dtype=self.dtype):
                pred_mask = self.predictor.gen_mask(
                    img_np=img_np,
                    frame_idx=frame_idx,
                    points=points,
                    labels=labels
                )
        return pred_mask
    
    
    def reset(self):
        self.predictor.init_inference_state()
        self.frame_idx = 0


    def segment(self, img, points, vis_name=None):
        N, _ = points.shape
        H, W, C = img.shape
        labels = np.ones([N], np.int32)

        old_h = self.predictor.video_height 
        old_w = self.predictor.video_width

        self.predictor.video_height = H
        self.predictor.video_width = W

        pred_mask = self.forward(
            img_np=img,
            frame_idx=0,
            points=points,
            labels=labels
        )

        # [NOTE] Deprecated, old plot function
        # self.save_visualization(
        #     img_np=img,
        #     pred_mask=pred_mask,
        #     frame_idx=0,
        #     points=points,
        #     labels=labels,
        #     file_name=vis_name
        # )
        mask_np = to_numpy_mask(pred_mask)
        if self.save_vis:
            show_fig(img, f"{self.vis_dir}/sam2_{vis_name}.png", masks=[mask_np], point_coords=points)

        self.reset()
        self.predictor.video_height = old_h
        self.predictor.video_width = old_w

        return mask2bbox(mask_np)
    

    def track(self, img, points=None, box=None):
        bbox, _mask = self.track_with_mask(img, points=points, box=box)
        return bbox

    def track_with_mask(self, img, points=None, box=None):
        track_started = time.perf_counter()
        labels = None
        if points is not None:
            N, _ = points.shape
            labels = np.ones([N], np.int32)

        if torch.cuda.is_available():
            torch.cuda.synchronize()
        inference_started = time.perf_counter()
        pred_mask = self.forward(
            img_np=img,
            frame_idx=self.frame_idx,
            points=points,
            labels=labels,
            box=box,
        )
        if torch.cuda.is_available():
            torch.cuda.synchronize()
        inference_s = time.perf_counter() - inference_started

        postprocess_started = time.perf_counter()
        mask_np = to_numpy_mask(pred_mask)
        bbox = mask2bbox(mask_np)
        postprocess_s = time.perf_counter() - postprocess_started
        # [NOTE] Deprecated, old plot function
        # self.save_visualization(
        #     img_np=img,
        #     pred_mask=pred_mask,
        #     frame_idx=self.frame_idx,
        #     points=points,
        #     labels=labels
        # )
        visualization_s = 0.0
        if self.save_vis:
            visualization_started = time.perf_counter()
            frame_number = f"{self.frame_idx:0{self.track_vis_index_width}d}"
            show_fig(
                img,
                f"{self.vis_dir}/{self.track_vis_prefix}_{frame_number}.png",
                masks=[mask_np],
                point_coords=points,
                box_coords=box,
            )
            visualization_s = time.perf_counter() - visualization_started

        self.frame_idx += 1
        self.last_timing = {
            "inference_s": inference_s,
            "postprocess_s": postprocess_s,
            "visualization_s": visualization_s,
            "total_s": time.perf_counter() - track_started,
        }

        return bbox, mask_np


    # [NOTE] Deprecated, old plot function
    # def save_visualization(
    #     self,
    #     img_np,
    #     pred_mask,
    #     frame_idx,
    #     points=None,
    #     labels=None,
    #     file_name=None,
    # ):
    #     plt.figure(figsize=(4, 3))
    #     plt.title(f"frame {frame_idx}")
    #     plt.imshow(img_np)

    #     if points is not None:
    #         self.show_points(points, labels, plt.gca())

    #     self.show_mask(
    #         pred_mask.cpu().numpy(),
    #         plt.gca(),
    #         obj_id=2
    #     )

    #     if file_name is None:
    #         save_path = f"{self.vis_dir}/sam_{frame_idx:03d}.png"
    #     else:
    #         save_path = f"{self.vis_dir}/sam_{file_name}_{frame_idx:03d}.png"

    #     plt.savefig(
    #         save_path,
    #         bbox_inches='tight',
    #         pad_inches=0
    #     )
    #     plt.close()
    #     print('[SAM-INFO] Saved in', save_path)


    # @staticmethod
    # def show_mask(mask, ax, obj_id=None, random_color=False):
    #     if random_color:
    #         color = np.concatenate([np.random.random(3), np.array([0.6])], axis=0)
    #     else:
    #         cmap = plt.get_cmap("tab10")
    #         cmap_idx = 0 if obj_id is None else obj_id
    #         color = np.array([*cmap(cmap_idx)[:3], 0.6])

    #     h, w = mask.shape[-2:]
    #     mask_image = mask.reshape(h, w, 1) * color.reshape(1, 1, -1)
    #     ax.imshow(mask_image)


    # @staticmethod
    # def show_points(coords, labels, ax, marker_size=200):
    #     pos_points = coords[labels == 1]
    #     neg_points = coords[labels == 0]
    #     ax.scatter(
    #         pos_points[:, 0],
    #         pos_points[:, 1],
    #         color='green',
    #         marker='*',
    #         s=marker_size,
    #         edgecolor='white',
    #         linewidth=1.25
    #     )
    #     ax.scatter(
    #         neg_points[:, 0],
    #         neg_points[:, 1],
    #         color='red',
    #         marker='*',
    #         s=marker_size,
    #         edgecolor='white',
    #         linewidth=1.25
    #     )


if __name__ == '__main__':
    cfg = SAM2Config()

    if os.path.exists(cfg.VIS_DIR):
        shutil.rmtree(cfg.VIS_DIR)
    os.makedirs(cfg.VIS_DIR, exist_ok=True)

    model = Sam2VideoPredictor(cfg)
    print("\n[INFO] Model initialized\n")

    all_imgs_path = []
    for name in os.listdir(cfg.VIDEO_FOLDER):
        path = os.path.join(cfg.VIDEO_FOLDER, name)
        all_imgs_path.append(path)
    all_imgs_path.sort()

    start = time.time()

    for frame_idx, img_path in enumerate(
        tqdm(all_imgs_path, desc='processing 1 scene')
    ):
        img_np = np.array(Image.open(img_path))
        points = None
        labels = None

        if frame_idx == 0:
            points = np.array( [[210, 350], [250, 220]], dtype=np.float32)
            labels = np.array([1, 1], np.int32)

        if frame_idx == 150:
            points = np.array([[82, 410]], dtype=np.float32)
            labels = np.array([0], np.int32)

        pred_mask = model.forward(
            img_np=img_np,
            frame_idx=frame_idx,
            points=points,
            labels=labels
        )

        if frame_idx % 30 == 0:
            # [NOTE] Deprecated, old plot function
            # model.save_visualization(
            #     img_np=img_np,
            #     pred_mask=pred_mask,
            #     frame_idx=frame_idx,
            #     points=points,
            #     labels=labels
            # )
            show_fig(img_np, f"{model.vis_dir}/sam2_{frame_idx:03d}.png", masks=[pred_mask], point_coords=points)


    print("\n[INFO] Finish")
    print("[INFO] Total time:", time.time() - start)
