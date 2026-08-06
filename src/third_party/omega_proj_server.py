from __future__ import annotations

import argparse
import glob
import importlib
import os
import shutil
import sys
import threading
from dataclasses import dataclass
from datetime import datetime
from typing import Annotated, List, Optional, Sequence

import cv2
import numpy as np
import torch


DEFAULT_SUBPROJ_ROOTS = [
    r"C:\Users\colab999\Desktop\project",
    r"C:\Users\colab999\Desktop\project\vggt_main",
    r"C:\Users\colab999\Desktop\project\omega_main",
]


def mcp_log(tool: str, msg: str):
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    print(f"[{now}][MCP][{tool}] {msg}")


def _add_subproj_roots(subproj_roots: Sequence[str]):
    """Add local project roots before importing VGGTOmega dependencies."""
    for sub_root in reversed(list(subproj_roots)):
        if sub_root and sub_root not in sys.path:
            sys.path.insert(0, sub_root)


@dataclass
class BestViewSelectorConfig:
    ckpt_path: str = r"C:\Users\colab999\Desktop\project\omega_main\vggt_omega_1b_512.pt"
    image_resolution: int = 320
    device: str = "cuda"
    save_vis: bool = True
    vis_dir: str = "vis_omega"
    subproj_roots: Optional[List[str]] = None


class BestViewSelector:
    def __init__(self, cfg: BestViewSelectorConfig):
        self.cfg = cfg
        self.vis_dir = cfg.vis_dir
        self._lock = threading.Lock()

        _add_subproj_roots(cfg.subproj_roots or DEFAULT_SUBPROJ_ROOTS)

        # Import after sys.path is prepared, so CLI arguments can override roots.
        from omega_main.vggt_omega.models import VGGTOmega
        from omega_main.vggt_omega.utils.pose_enc import encoding_to_camera
        from vggt_main.vggt.utils.geometry import unproject_depth_map_to_point_map
        from utils import show_fig

        self._encoding_to_camera = encoding_to_camera
        self._unproject_depth_map_to_point_map = unproject_depth_map_to_point_map
        self._show_fig = show_fig

        self.model = VGGTOmega().to(cfg.device).eval()
        state_dict = torch.load(cfg.ckpt_path, map_location="cpu")
        self.model.load_state_dict(state_dict)

    def set_vis_dir(self, vis_dir: str):
        self.vis_dir = vis_dir
        self.cfg.vis_dir = vis_dir

    def _resize_and_pad_square(
        self,
        img: np.ndarray,
        points: Optional[np.ndarray],  # N, 2
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
        imgs: List[np.ndarray],  # B, H, W, C
        points: np.ndarray,  # N, 2
    ):
        batch_img = []
        metas = []
        res_points = None
        for i, img in enumerate(imgs):
            new_img, new_points, meta = self._resize_and_pad_square(
                img,
                points if i == 0 else None,
                fixed_size=self.cfg.image_resolution,
            )
            batch_img.append(new_img)
            if i == 0:
                res_points = new_points
            metas.append(meta)

        batch_img = np.stack(batch_img)
        return batch_img, res_points, metas

    def _restore_points(
        self,
        points: np.ndarray,  # N, 2
        meta,
    ):
        points_out = points.astype(np.float32).copy()
        points_out[:, 0] -= meta["pad_left"]
        points_out[:, 1] -= meta["pad_top"]
        points_out /= meta["scale"]
        return points_out

    @torch.inference_mode()
    def _run_vggt(self, images: np.ndarray):
        images = torch.from_numpy(images).permute(0, 3, 1, 2).float() / 255.0
        images = images.to(self.cfg.device)
        predictions = self.model(images)
        extrinsics, intrinsics = self._encoding_to_camera(
            predictions["pose_enc"],
            predictions["images"].shape[-2:],
        )
        return {
            "depth": predictions["depth"].detach().cpu().numpy(),
            "extrinsics": extrinsics.detach().cpu().numpy(),
            "intrinsics": intrinsics.detach().cpu().numpy(),
        }

    def _project_world_points(
        self,
        points_xyz: np.ndarray,  # (N,3)
        extrinsic: np.ndarray,  # (3,4)
        intrinsic: np.ndarray,  # (3,3)
    ):
        R = extrinsic[:, :3]
        t = extrinsic[:, 3]

        # world -> camera
        points_cam = points_xyz @ R.T + t
        z = points_cam[:, 2]

        # keep only points in front of camera
        valid_mask = z > 0
        if not np.any(valid_mask):
            return None, None

        points_cam = points_cam[valid_mask]
        # camera -> image
        uv = points_cam @ intrinsic.T
        u = uv[:, 0] / uv[:, 2]
        v = uv[:, 1] / uv[:, 2]
        points_2d = np.stack([u, v], axis=1)

        return points_2d.astype(np.int32), valid_mask

    def _find_best_view(
        self,
        points_xyz: np.ndarray,
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

    def select_view(
        self,
        src_img: np.ndarray,  # H, W, C
        tar_imgs: List[np.ndarray],  # B, H, W, C
        src_points: np.ndarray,  # N, 2
        suffix: str = "select",
    ):
        if src_img is None:
            raise ValueError("src_img is None")
        if not tar_imgs:
            raise ValueError("tar_imgs must contain at least one target image")
        if src_points.ndim != 2 or src_points.shape[1] != 2:
            raise ValueError("src_points must have shape (N, 2)")

        batch_img, points, metas = self._preprocess([src_img] + tar_imgs, src_points)
        if self.cfg.save_vis:
            os.makedirs(self.vis_dir, exist_ok=True)
            self._show_fig(
                src_img,
                os.path.join(self.vis_dir, f"omega_query_{suffix}.png"),
                point_coords=src_points,
            )

        outputs = self._run_vggt(batch_img)
        extrinsics = outputs["extrinsics"]
        intrinsics = outputs["intrinsics"]
        depth = outputs["depth"]

        point_map_world = self._unproject_depth_map_to_point_map(
            depth[0, 0:1],
            extrinsics[0, 0:1],
            intrinsics[0, 0:1],
        )[0]

        points_int = np.round(points).astype(np.int32)
        h, w = point_map_world.shape[:2]
        xs = np.clip(points_int[:, 0], 0, w - 1)
        ys = np.clip(points_int[:, 1], 0, h - 1)
        points_xyz = point_map_world[ys, xs]

        Rs = extrinsics[0][1:, :, :3]
        ts = extrinsics[0][1:, :, 3]

        best_idx, angles_deg = self._find_best_view(points_xyz, Rs, ts)
        best_points, valid_mask = self._project_world_points(
            points_xyz,
            extrinsics[0, best_idx + 1],
            intrinsics[0, best_idx + 1],
        )
        if best_points is None:
            raise RuntimeError("No source points were projected in front of the selected target camera")

        best_points_ori = self._restore_points(
            best_points,
            metas[best_idx + 1],
        )
        if self.cfg.save_vis:
            os.makedirs(self.vis_dir, exist_ok=True)
            self._show_fig(
                tar_imgs[best_idx],
                os.path.join(self.vis_dir, f"omega_target_{suffix}.png"),
                point_coords=best_points_ori,
            )

        return best_idx, best_points_ori, angles_deg, valid_mask


def _load_image(path: str) -> np.ndarray:
    img = cv2.imread(path)
    if img is None:
        raise FileNotFoundError(f"failed to read image: {path}")
    return img


def _coerce_points(points: Sequence[Sequence[float]]) -> np.ndarray:
    arr = np.asarray(points, dtype=np.float32)
    if arr.ndim != 2 or arr.shape[1] != 2:
        raise ValueError("src_points must be a list like [[x, y], [x2, y2], ...]")
    return arr


def _expand_image_paths(paths_or_globs: Sequence[str]) -> List[str]:
    expanded: List[str] = []
    for item in paths_or_globs:
        matches = sorted(glob.glob(item))
        if matches:
            expanded.extend(matches)
        else:
            expanded.append(item)
    return expanded


def _schedule_process_shutdown(delay_seconds: float = 0.5, exit_code: int = 0):
    """Terminate this MCP server process after the tool response has been sent."""
    delay_seconds = max(0.1, float(delay_seconds))

    def _shutdown():
        mcp_log("omega_shutdown", f"Exiting process with code={exit_code}.")
        os._exit(int(exit_code))

    timer = threading.Timer(delay_seconds, _shutdown)
    timer.daemon = True
    timer.start()
    return {"ok": True, "message": f"Omega MCP server will shut down in {delay_seconds:.1f}s", "exit_code": int(exit_code)}


def _create_fastmcp(selector: BestViewSelector):
    try:
        fastmcp_module = importlib.import_module("fastmcp")
        FastMCP = fastmcp_module.FastMCP
    except Exception as exc:
        raise RuntimeError("fastmcp is not installed") from exc

    mcp = FastMCP("omega_projector")

    @mcp.tool(
        name="health",
        description="Return Omega best-view selector model and runtime configuration status.",
    )
    def mcp_health() -> dict:
        """Return Omega best-view selector model and runtime configuration status."""
        health = {
            "ckpt_path": selector.cfg.ckpt_path,
            "image_resolution": selector.cfg.image_resolution,
            "device": selector.cfg.device,
            "save_vis": selector.cfg.save_vis,
            "vis_dir": selector.vis_dir,
            "model_loaded": selector.model is not None,
        }
        mcp_log("omega_health", f"Health State={health}.")
        return {"ok": True, "health": health}

    @mcp.tool(
        name="set_vis_dir",
        description="Set the directory used to save Omega query and target visualization images.",
    )
    def mcp_set_vis_dir(
        vis_dir: Annotated[str, "Directory path for saved visualization images."],
    ) -> dict:
        """Set the directory used to save Omega query and target visualization images."""
        if os.path.exists(vis_dir):
            shutil.rmtree(vis_dir)
        os.makedirs(vis_dir, exist_ok=True)
        selector.set_vis_dir(vis_dir)
        mcp_log("omega_set_vis_dir", f"vis_dir={vis_dir}")
        return {"ok": True, "vis_dir": vis_dir}

    @mcp.tool(
        name="select_view",
        description=(
            "Select the best target image view for source image points. "
            "Inputs are local image paths and source points [[x, y], ...]."
        ),
    )
    def mcp_select_view(
        src_img_path: Annotated[str, "Local path to the source/query image."],
        tar_img_paths: Annotated[List[str], "Local paths or glob patterns for candidate target images."],
        src_points: Annotated[List[List[float]], "Source points in original source image coordinates, e.g. [[300, 200]]."],
        suffix: Annotated[str, "Suffix used in saved visualization filenames."] = "select",
        save_vis: Annotated[Optional[bool], "Override whether visualizations are saved for this call. Use null to keep current config."] = None,
    ) -> dict:
        """Select the best target image view for source image points."""
        mcp_log(
            "omega_select_view",
            f"src={src_img_path}, num_targets={len(tar_img_paths)}, src_points={src_points}, suffix={suffix}",
        )

        with selector._lock:
            old_save_vis = selector.cfg.save_vis
            if save_vis is not None:
                selector.cfg.save_vis = bool(save_vis)
            try:
                expanded_paths = _expand_image_paths(tar_img_paths)
                src_img = _load_image(src_img_path)
                tar_imgs = [_load_image(path) for path in expanded_paths]
                points = _coerce_points(src_points)

                best_idx, best_points_ori, angles_deg, valid_mask = selector.select_view(
                    src_img=src_img,
                    tar_imgs=tar_imgs,
                    src_points=points,
                    suffix=suffix,
                )

                result = {
                    "ok": True,
                    "best_idx": int(best_idx),
                    "best_target_path": expanded_paths[int(best_idx)],
                    "best_points": best_points_ori.astype(float).tolist(),
                    "angles_deg": angles_deg.astype(float).tolist(),
                    "valid_mask": None if valid_mask is None else valid_mask.astype(bool).tolist(),
                    "num_targets": len(expanded_paths),
                    "vis_dir": selector.vis_dir,
                    "saved_visualizations": {
                        "query": os.path.join(selector.vis_dir, f"omega_query_{suffix}.png"),
                        "target": os.path.join(selector.vis_dir, f"omega_target_{suffix}.png"),
                    } if selector.cfg.save_vis else None,
                }
                mcp_log(
                    "omega_select_view",
                    f"best_idx={result['best_idx']}, best_target={result['best_target_path']}",
                )
                return result
            finally:
                selector.cfg.save_vis = old_save_vis


    @mcp.tool(
        name="shutdown",
        description=(
            "Shut down the Omega MCP server process. The process exits shortly after "
            "this tool returns so the client can receive the response."
        ),
    )
    def mcp_shutdown(
        delay_seconds: Annotated[float, "Seconds to wait before terminating the server process after returning the response."] = 0.5,
        exit_code: Annotated[int, "Process exit code to use when shutting down."] = 0,
    ) -> dict:
        """Shut down the Omega MCP server process after returning a response."""
        mcp_log("omega_shutdown", f"Shutdown requested: delay_seconds={delay_seconds}, exit_code={exit_code}")
        return _schedule_process_shutdown(delay_seconds=delay_seconds, exit_code=exit_code)

    return mcp


def _run_fastmcp(
    selector: BestViewSelector,
    transport: str,
    host: str,
    port: int,
    block: bool = True,
):
    mcp = _create_fastmcp(selector)

    def _serve():
        kwargs = {"transport": transport}
        if transport != "stdio":
            kwargs["host"] = host
            kwargs["port"] = port
        try:
            mcp.run(**kwargs)
        except TypeError:
            # Compatibility with older FastMCP versions that accept no keyword args.
            mcp.run()

    if block:
        _serve()
        return None

    thread = threading.Thread(target=_serve, daemon=True)
    thread.start()
    return thread


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Omega best-view selector MCP server")
    parser.add_argument("--ckpt-path", default=BestViewSelectorConfig.ckpt_path)
    parser.add_argument("--image-resolution", type=int, default=BestViewSelectorConfig.image_resolution)
    parser.add_argument("--device", default=BestViewSelectorConfig.device)
    parser.add_argument("--save-vis", action=argparse.BooleanOptionalAction, default=BestViewSelectorConfig.save_vis)
    parser.add_argument("--vis-dir", default=BestViewSelectorConfig.vis_dir)
    parser.add_argument(
        "--subproj-root",
        action="append",
        default=None,
        help="Project root to prepend to sys.path. Can be provided multiple times. Defaults to the original hard-coded project roots.",
    )
    parser.add_argument("--fastmcp-transport", default="streamable-http", choices=["sse", "streamable-http", "stdio"])
    parser.add_argument("--fastmcp-host", default="127.0.0.1")
    parser.add_argument("--fastmcp-port", type=int, default=8757)
    return parser


def main():
    parser = _build_arg_parser()
    args = parser.parse_args()

    cfg = BestViewSelectorConfig(
        ckpt_path=args.ckpt_path,
        image_resolution=args.image_resolution,
        device=args.device,
        save_vis=args.save_vis,
        vis_dir=args.vis_dir,
        subproj_roots=args.subproj_root,
    )

    if os.path.exists(cfg.vis_dir):
        shutil.rmtree(cfg.vis_dir)
    os.makedirs(cfg.vis_dir, exist_ok=True)

    selector = BestViewSelector(cfg)

    print(
        f"FastMCP ready: transport={args.fastmcp_transport}, "
        f"host={args.fastmcp_host}, port={args.fastmcp_port}"
    )
    _run_fastmcp(
        selector,
        transport=args.fastmcp_transport,
        host=args.fastmcp_host,
        port=args.fastmcp_port,
        block=True,
    )


if __name__ == "__main__":
    main()
