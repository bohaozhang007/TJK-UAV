import os
import shutil
import sys
subproj_roots = [
    r'C:\Users\colab999\Desktop\project',
    r'C:\Users\colab999\Desktop\project\vggt_main',
    r'C:\Users\colab999\Desktop\project\omega_main',
]
for sub_root in subproj_roots:
    sys.path.insert(0, sub_root)

from typing import List
import numpy as np
import cv2
import torch
from dataclasses import dataclass

from omega_main.vggt_omega.models import VGGTOmega
from omega_main.vggt_omega.utils.pose_enc import encoding_to_camera
from vggt_main.vggt.utils.geometry import unproject_depth_map_to_point_map

from utils import show_fig


@dataclass
class BestViewSelectorConfig:
    ckpt_path: str = r"C:\Users\colab999\Desktop\project\omega_main\vggt_omega_1b_512.pt"
    image_resolution: int = 512
    device: str = "cuda"
    save_vis = True
    VIS_DIR = 'vis_omega'


class BestViewSelector:
    def __init__(self, cfg: BestViewSelectorConfig):
        self.cfg = cfg
        self.model = VGGTOmega().to(cfg.device).eval()
        state_dict = torch.load(cfg.ckpt_path, map_location="cpu")
        self.model.load_state_dict(state_dict)
        self.vis_dir = cfg.VIS_DIR

    def set_vis_dir(
            self,
            vis_dir
    ):
        self.vis_dir = vis_dir

    def _resize_and_pad_square(
            self,
            img: np.ndarray,
            points: np.ndarray, # N, 2
            fixed_size: int,
    ):
        h, w = img.shape[:2]
        scale = fixed_size / max(h, w)
        new_h = int(round(h * scale))
        new_w = int(round(w * scale))

        img_resize = cv2.resize(
            img,
            (new_w, new_h),
            interpolation=cv2.INTER_LINEAR,
        )

        pad_top = (fixed_size - new_h) // 2
        pad_bottom = fixed_size - new_h - pad_top
        pad_left = (fixed_size - new_w) // 2
        pad_right = fixed_size - new_w - pad_left

        img_out = cv2.copyMakeBorder(
            img_resize,
            pad_top,
            pad_bottom,
            pad_left,
            pad_right,
            borderType=cv2.BORDER_CONSTANT,
            value=(255, 255, 255),
        )

        if points is not None:
            points_out = points.astype(np.float32).copy()
            points_out *= scale
            points_out[:, 0] += pad_left
            points_out[:, 1] += pad_top
        else:
            points_out = None

        meta = {
            "scale": scale,
            "pad_left": pad_left,
            "pad_top": pad_top,
            "ori_h": h,
            "ori_w": w,
        }
        return img_out, points_out, meta
    
    def _preprocess(
            self,
            imgs: List[np.ndarray], # B, H, W, C
            points: np.ndarray #  N, 2
    ):
        batch_img = []
        metas = []
        res_points = None
        for i, img in enumerate(imgs):
            new_img, new_points, meta = self._resize_and_pad_square(img, points if i == 0 else None, fixed_size=self.cfg.image_resolution)
            batch_img.append(new_img)
            if i == 0:
                res_points = new_points 
            metas.append(meta)

        batch_img = np.stack(batch_img)
        return batch_img, res_points, metas
    
    def _restore_points(
            self,
            points, # N, 2
            meta,
    ):
        points_out = points.astype(np.float32).copy()
        points_out[:, 0] -= meta["pad_left"]
        points_out[:, 1] -= meta["pad_top"]
        points_out /= meta["scale"]
        return points_out
        
    @torch.inference_mode()
    def _run_vggt(
        self,
        images,
    ):
        images = torch.from_numpy(images).permute(0, 3, 1, 2).float() / 255.0
        images = images.to(self.cfg.device)
        predictions = self.model(images)
        extrinsics, intrinsics = encoding_to_camera(
            predictions["pose_enc"],
            predictions["images"].shape[-2:],
        )
        return {
            "depth": predictions["depth"].detach().cpu().numpy(), #torch.Size([1, 13, 416, 624, 1])
            "extrinsics": extrinsics.detach().cpu().numpy(), # torch.Size([1, 13, 3, 4])
            "intrinsics": intrinsics.detach().cpu().numpy(), # torch.Size([1, 13, 3, 3])
        }
    
    # [NOTE] Deprecated, only 1 point situation
    # def _project_world_point(
    #         self,
    #         point_xyz: np.ndarray, # (3,)
    #         extrinsic: np.ndarray, # (3,4)
    #         intrinsic: np.ndarray, # (3,3)
    # ):
    #     R = extrinsic[:, :3]
    #     t = extrinsic[:, 3]
    #     point_cam = R @ point_xyz + t
    #     z = point_cam[2]
    #     if z <= 0:
    #         return None
    #     uv = intrinsic @ point_cam
    #     u = uv[0] / uv[2]
    #     v = uv[1] / uv[2]
    #     return np.array([u, v], dtype=np.int32)


    def _project_world_points(
        self,
        points_xyz: np.ndarray,  # (N,3)
        extrinsic: np.ndarray,   # (3,4)
        intrinsic: np.ndarray,   # (3,3)
    ):
        R = extrinsic[:, :3]      # (3,3)
        t = extrinsic[:, 3]       # (3,)

        # world -> camera
        points_cam = points_xyz @ R.T + t    # (N,3)
        z = points_cam[:, 2]                 # (N,)

        # keep only points in front of camera
        valid_mask = z > 0
        if not np.any(valid_mask):
            return None, None

        points_cam = points_cam[valid_mask]
        # camera -> image
        uv = points_cam @ intrinsic.T        # (M,3)
        u = uv[:, 0] / uv[:, 2]
        v = uv[:, 1] / uv[:, 2]
        points_2d = np.stack([u, v], axis=1)

        return points_2d.astype(np.int32), valid_mask

    def _find_best_view(
        self,
        points_xyz:np.ndarray,
        drone_Rs: np.ndarray,
        drone_ts: np.ndarray,
    ):
        angles_deg = []
        obj_center_world = points_xyz.mean(0)

        for R, t in zip(drone_Rs, drone_ts):
            # camera center in world
            C = -R.T @ t
            # camera -> object
            d = obj_center_world - C
            d /= np.linalg.norm(d)
            # camera optical axis in world
            optical_axis = R.T @ np.array([0.0, 0.0, 1.0])
            optical_axis /= np.linalg.norm(optical_axis)
            cos_theta = np.clip(
                np.dot(optical_axis, d),
                -1.0,
                1.0,
            )
            theta = np.degrees(np.arccos(cos_theta))
            angles_deg.append(theta)

        angles_deg = np.asarray(angles_deg)
        best_idx = int(np.argmin(angles_deg))
        return best_idx, angles_deg
    
    # [NOTE] Deprecated, old plot function
    # def _save_debug_points(
    #     self,
    #     img: np.ndarray,
    #     points: np.ndarray,
    #     save_name: str,
    # ):
    #     img = img.copy()
    #     points = np.asarray(points).astype(np.int32)
    #     for x, y in points:
    #         cv2.circle(
    #             img,
    #             (x, y),
    #             radius=8,
    #             color=(0, 0, 255),
    #             thickness=-1,
    #         )

    #     cv2.imwrite(f"{self.vis_dir}/omega_{save_name}.png", img)

    def select_view(
        self,
        src_img: np.ndarray, # H, W, C
        tar_imgs: List[np.ndarray], # B, H, W, C
        src_points: np.ndarray, # N, 2
        suffix = 'select'
    ) -> int:
        batch_img, points, metas = self._preprocess([src_img] + tar_imgs, src_points)
        if self.cfg.save_vis:
            # self._save_debug_points(batch_img[0], points, f"{suffix}")
            # self._save_debug_points(src_img, src_points, f"{suffix}")
            show_fig(src_img, f"{self.vis_dir}/omega_query_{suffix}.png", point_coords=src_points)

        outputs = self._run_vggt(batch_img)
        extrinsics = outputs["extrinsics"]
        intrinsics = outputs["intrinsics"]
        depth = outputs["depth"]

        point_map_world = unproject_depth_map_to_point_map(
            depth[0, 0:1], # (1,H,W,1)
            extrinsics[0, 0:1], # (1,3,4)
            intrinsics[0, 0:1], # (1,3,3)
        )[0] # (H,W,3)

        # [NOTE] Deprecated, only 1 point situation
        # x, y = np.round(points[0]).astype(np.int32)
        # point_xyz = point_map_world[y, x]

        points_int = np.round(points).astype(np.int32)
        xs = points_int[:, 0]
        ys = points_int[:, 1]
        points_xyz = point_map_world[ys, xs]

        Rs = extrinsics[0][1:, :, :3]
        ts = extrinsics[0][1:, :, 3]

        best_idx, angles_deg = self._find_best_view(points_xyz, Rs, ts)
        best_points, valid_mask = self._project_world_points(
            points_xyz,
            extrinsics[0, best_idx + 1],
            intrinsics[0, best_idx + 1],
        )

        best_points_ori = self._restore_points(
            best_points,
            metas[best_idx + 1],
        )
        if self.cfg.save_vis:
            # self._save_debug_points(batch_img[best_idx+1], best_points, f"tar_{suffix}")
            # self._save_debug_points(tar_imgs[best_idx], best_points_ori, f"tar_{suffix}")
            show_fig(tar_imgs[best_idx], f"{self.vis_dir}/omega_target_{suffix}.png", point_coords=best_points_ori)

        return best_idx, best_points_ori, angles_deg 
    

if __name__ == "__main__":
    cfg = BestViewSelectorConfig()
    selector = BestViewSelector(cfg)

    if os.path.exists(cfg.VIS_DIR):
        shutil.rmtree(cfg.VIS_DIR)
    os.makedirs(cfg.VIS_DIR, exist_ok=True)


    src_img = cv2.imread(
        r"C:\Users\colab999\Desktop\project\vggt_main\examples\room\images\no_overlap_8.jpg"
    )
    tar_imgs = [
        cv2.imread(r"C:\Users\colab999\Desktop\project\vggt_main\examples\room\images\no_overlap_1.png"),
        cv2.imread(r"C:\Users\colab999\Desktop\project\vggt_main\examples\room\images\no_overlap_1.png"),
        cv2.imread(r"C:\Users\colab999\Desktop\project\vggt_main\examples\room\images\no_overlap_2.jpg"),
        cv2.imread(r"C:\Users\colab999\Desktop\project\vggt_main\examples\room\images\no_overlap_3.jpg"),
        cv2.imread(r"C:\Users\colab999\Desktop\project\vggt_main\examples\room\images\no_overlap_4.jpg"),
        cv2.imread(r"C:\Users\colab999\Desktop\project\vggt_main\examples\room\images\no_overlap_5.jpg"),
        cv2.imread(r"C:\Users\colab999\Desktop\project\vggt_main\examples\room\images\no_overlap_6.jpg"),
        cv2.imread(r"C:\Users\colab999\Desktop\project\vggt_main\examples\room\images\no_overlap_7.jpg"),
    ]

    src_points = np.array([[300, 200]])
    best_idx, best_point, angles_deg = selector.select_view(
        src_img=src_img,
        tar_imgs=tar_imgs,
        src_points=src_points,
    )

    print("\nCandidate scores:")
    for i, deg in enumerate(angles_deg):
        print(f"[{i}] {deg:7.3f}°")

    print("\nBest View:")
    print(f"index = {best_idx}")