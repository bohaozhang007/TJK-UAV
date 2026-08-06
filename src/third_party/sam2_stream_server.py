from __future__ import annotations

import argparse
import importlib
import os
import shutil
import sys
import threading
from dataclasses import dataclass
from datetime import datetime
from typing import Annotated, List, Optional, Sequence

import hydra
import matplotlib
matplotlib.use("Agg")  # Don't show img in the call
import numpy as np
import torch
from hydra.core.global_hydra import GlobalHydra
from PIL import Image


DEFAULT_SUBPROJ_ROOTS = [
    r"C:\Users\colab999\Desktop\project",
    r"C:\Users\colab999\Desktop\project\Grounded_SAM_2_main",
]


def mcp_log(tool: str, msg: str):
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    print(f"[{now}][MCP][{tool}] {msg}")


def _add_subproj_roots(subproj_roots: Sequence[str]):
    """Add local project roots before importing Grounded-SAM-2 dependencies."""
    for sub_root in reversed(list(subproj_roots)):
        if sub_root and sub_root not in sys.path:
            sys.path.insert(0, sub_root)


def _resolve_torch_dtype(dtype_name: str):
    name = str(dtype_name).lower().replace("torch.", "")
    mapping = {
        "bfloat16": torch.bfloat16,
        "bf16": torch.bfloat16,
        "float16": torch.float16,
        "fp16": torch.float16,
        "float32": torch.float32,
        "fp32": torch.float32,
    }
    if name not in mapping:
        raise ValueError(f"unsupported dtype: {dtype_name}; use bfloat16, float16, or float32")
    return mapping[name]


def _mask_to_numpy(mask) -> np.ndarray:
    if isinstance(mask, torch.Tensor):
        mask = mask.detach().cpu().numpy()
    mask = np.asarray(mask)
    # Common SAM outputs may contain singleton dimensions such as (1, H, W).
    mask = np.squeeze(mask)
    return mask


def mask2bbox(mask):
    mask_np = _mask_to_numpy(mask)
    ys, xs = np.where(mask_np > 0)
    if len(xs) == 0 or len(ys) == 0:
        return None
    x1, x2 = xs.min(), xs.max()
    y1, y2 = ys.min(), ys.max()
    bbox = np.array([[x1, y1], [x2, y2]], dtype=np.int32)
    return bbox


@dataclass
class SAM2Config:
    sam2_cfg: str = "sam2.1/sam2.1_hiera_s.yaml"
    sam2_ckpt: str = r"C:\Users\colab999\Desktop\project\Grounded_SAM_2_main\checkpoints\sam2.1_hiera_small.pt"
    video_folder: str = r"C:\Users\colab999\Desktop\project\Grounded_SAM_2_main\notebooks\videos\bedroom"
    hydra_config: str = r"C:\Users\colab999\Desktop\project\Grounded_SAM_2_main\sam2\configs"
    local_rank: int = 0
    device: str = "cuda"
    dtype: str = "bfloat16"
    video_height: int = 240
    video_width: int = 320
    vis_dir: str = "vis_sam2"
    subproj_roots: Optional[List[str]] = None


class Sam2VideoPredictor:
    def __init__(self, cfg: SAM2Config):
        self.cfg = cfg
        self.device = cfg.device
        self.dtype = _resolve_torch_dtype(cfg.dtype)
        self.vis_dir = cfg.vis_dir
        self.frame_idx = 0
        self._lock = threading.Lock()

        os.environ["PYTORCH_CUDA_ALLOC_CONF"] = "expandable_segments:True"
        _add_subproj_roots(cfg.subproj_roots or DEFAULT_SUBPROJ_ROOTS)

        # Import after sys.path is prepared, so CLI arguments can override roots.
        from Grounded_SAM_2_main.sam2.build_sam import build_one_obj_video_predictor
        from utils import show_fig

        self._show_fig = show_fig

        print(f"[SAM-INFO] SAM Using device: {self.device}")
        GlobalHydra.instance().clear()
        hydra.initialize_config_dir(config_dir=cfg.hydra_config, version_base=None)

        self.predictor = build_one_obj_video_predictor(
            cfg.sam2_cfg,
            cfg.sam2_ckpt,
            device=self.device,
        )
        print("[SAM-INFO] Finish loading SAM")

        self.predictor.init_inference_state()
        self.predictor.video_height = cfg.video_height
        self.predictor.video_width = cfg.video_width

    def set_vis_dir(self, vis_dir: str):
        self.vis_dir = vis_dir
        self.cfg.vis_dir = vis_dir

    def set_img_size(self, img_H: int, img_W: int):
        self.predictor.video_height = int(img_H)
        self.predictor.video_width = int(img_W)
        self.cfg.video_height = int(img_H)
        self.cfg.video_width = int(img_W)

    def forward(
        self,
        img_np: np.ndarray,
        frame_idx: int,
        points: Optional[np.ndarray] = None,
        labels: Optional[np.ndarray] = None,
    ):
        with torch.inference_mode():
            if self.device.startswith("cuda"):
                with torch.amp.autocast("cuda", dtype=self.dtype):
                    pred_mask = self.predictor.gen_mask(
                        img_np=img_np,
                        frame_idx=frame_idx,
                        points=points,
                        labels=labels,
                    )
            else:
                pred_mask = self.predictor.gen_mask(
                    img_np=img_np,
                    frame_idx=frame_idx,
                    points=points,
                    labels=labels,
                )
        return pred_mask

    def reset(self):
        self.predictor.init_inference_state()
        self.frame_idx = 0

    def segment(
        self,
        img: np.ndarray,
        points: np.ndarray,
        labels: Optional[np.ndarray] = None,
        vis_name: Optional[str] = None,
        save_vis: bool = True,
    ):
        if img is None:
            raise ValueError("img is None")
        if points.ndim != 2 or points.shape[1] != 2:
            raise ValueError("points must have shape (N, 2)")

        n, _ = points.shape
        if labels is None:
            labels = np.ones([n], np.int32)
        else:
            labels = np.asarray(labels, dtype=np.int32)
            if labels.shape != (n,):
                raise ValueError("labels must have shape (N,) and match points")

        h, w, _ = img.shape
        old_h = self.predictor.video_height
        old_w = self.predictor.video_width

        self.predictor.video_height = h
        self.predictor.video_width = w

        pred_mask = self.forward(
            img_np=img,
            frame_idx=0,
            points=points,
            labels=labels,
        )

        vis_path = None
        if save_vis:
            os.makedirs(self.vis_dir, exist_ok=True)
            safe_name = vis_name or "segment"
            vis_path = os.path.join(self.vis_dir, f"sam2_{safe_name}.png")
            self._show_fig(img, vis_path, masks=[pred_mask], point_coords=points)

        self.reset()
        self.predictor.video_height = old_h
        self.predictor.video_width = old_w

        return mask2bbox(pred_mask), vis_path

    def track(
        self,
        img: np.ndarray,
        points: Optional[np.ndarray] = None,
        labels: Optional[np.ndarray] = None,
        save_vis: bool = True,
    ):
        if img is None:
            raise ValueError("img is None")

        if points is not None:
            if points.ndim != 2 or points.shape[1] != 2:
                raise ValueError("points must have shape (N, 2)")
            n, _ = points.shape
            if labels is None:
                labels = np.ones([n], np.int32)
            else:
                labels = np.asarray(labels, dtype=np.int32)
                if labels.shape != (n,):
                    raise ValueError("labels must have shape (N,) and match points")
        else:
            labels = None

        frame_idx = self.frame_idx
        pred_mask = self.forward(
            img_np=img,
            frame_idx=frame_idx,
            points=points,
            labels=labels,
        )

        vis_path = None
        if save_vis:
            os.makedirs(self.vis_dir, exist_ok=True)
            vis_path = os.path.join(self.vis_dir, f"sam2_track_{frame_idx}.png")
            self._show_fig(img, vis_path, masks=[pred_mask], point_coords=points)

        self.frame_idx += 1
        return mask2bbox(pred_mask), vis_path, frame_idx



def _load_image(path: str) -> np.ndarray:
    if not path:
        raise ValueError("image path is empty")
    if not os.path.exists(path):
        raise FileNotFoundError(f"image not found: {path}")
    return np.array(Image.open(path).convert("RGB"))


def _coerce_points(points: Optional[Sequence[Sequence[float]]]) -> Optional[np.ndarray]:
    if points is None:
        return None
    arr = np.asarray(points, dtype=np.float32)
    if arr.ndim != 2 or arr.shape[1] != 2:
        raise ValueError("points must be a list like [[x, y], [x2, y2], ...]")
    return arr


def _coerce_labels(labels: Optional[Sequence[int]], expected_len: Optional[int] = None) -> Optional[np.ndarray]:
    if labels is None:
        return None
    arr = np.asarray(labels, dtype=np.int32)
    if arr.ndim != 1:
        raise ValueError("labels must be a list like [1, 1, 0]")
    if expected_len is not None and arr.shape[0] != expected_len:
        raise ValueError("labels length must match points length")
    return arr


def _bbox_to_list(bbox) -> Optional[List[List[int]]]:
    if bbox is None:
        return None
    return np.asarray(bbox, dtype=np.int32).tolist()


def _schedule_process_shutdown(delay_seconds: float = 0.5, exit_code: int = 0):
    """Terminate this MCP server process after the tool response has been sent."""
    delay_seconds = max(0.1, float(delay_seconds))

    def _shutdown():
        mcp_log("sam2_shutdown", f"Exiting process with code={exit_code}.")
        os._exit(int(exit_code))

    timer = threading.Timer(delay_seconds, _shutdown)
    timer.daemon = True
    timer.start()
    return {
        "ok": True,
        "message": f"SAM2 MCP server will shut down in {delay_seconds:.1f}s",
        "exit_code": int(exit_code),
    }


def _create_fastmcp(model: Sam2VideoPredictor):
    try:
        fastmcp_module = importlib.import_module("fastmcp")
        FastMCP = fastmcp_module.FastMCP
    except Exception as exc:
        raise RuntimeError("fastmcp is not installed") from exc

    mcp = FastMCP("sam2")

    @mcp.tool(
        name="health",
        description="Return SAM2 video predictor model and runtime configuration status.",
    )
    def mcp_health() -> dict:
        """Return SAM2 video predictor model and runtime configuration status."""
        health = {
            "sam2_cfg": model.cfg.sam2_cfg,
            "sam2_ckpt": model.cfg.sam2_ckpt,
            "hydra_config": model.cfg.hydra_config,
            "device": model.cfg.device,
            "dtype": model.cfg.dtype,
            "video_height": int(model.predictor.video_height),
            "video_width": int(model.predictor.video_width),
            "vis_dir": model.vis_dir,
            "frame_idx": int(model.frame_idx),
            "model_loaded": model.predictor is not None,
        }
        mcp_log("sam2_health", f"Health State={health}.")
        return {"ok": True, "health": health}

    @mcp.tool(
        name="set_vis_dir",
        description="Set the directory used to save SAM2 visualization images.",
    )
    def mcp_set_vis_dir(
        vis_dir: Annotated[str, "Directory path for saved visualization images."],
        clear_existing: Annotated[bool, "Whether to remove the directory first if it already exists."] = False,
    ) -> dict:
        """Set the directory used to save SAM2 visualization images."""
        with model._lock:
            if clear_existing and os.path.exists(vis_dir):
                shutil.rmtree(vis_dir)
            os.makedirs(vis_dir, exist_ok=True)
            model.set_vis_dir(vis_dir)
        mcp_log("sam2_set_vis_dir", f"vis_dir={vis_dir}, clear_existing={clear_existing}")
        return {"ok": True, "vis_dir": vis_dir}

    @mcp.tool(
        name="set_img_size",
        description="Set the default video frame size used by SAM2 tracking.",
    )
    def mcp_set_img_size(
        img_H: Annotated[int, "Image height in pixels."],
        img_W: Annotated[int, "Image width in pixels."],
    ) -> dict:
        """Set the default video frame size used by SAM2 tracking."""
        with model._lock:
            model.set_img_size(img_H, img_W)
        mcp_log("sam2_set_img_size", f"img_H={img_H}, img_W={img_W}")
        return {"ok": True, "video_height": int(img_H), "video_width": int(img_W)}

    @mcp.tool(
        name="reset",
        description="Reset SAM2 streaming inference state and frame index.",
    )
    def mcp_reset() -> dict:
        """Reset SAM2 streaming inference state and frame index."""
        with model._lock:
            model.reset()
        mcp_log("sam2_reset", "Inference state reset.")
        return {"ok": True, "frame_idx": int(model.frame_idx)}

    @mcp.tool(
        name="segment",
        description=(
            "Segment an object in one image from point prompts. "
            "Inputs are a local image path, points [[x, y], ...], and optional labels. "
            "Returns a bounding box [[x1, y1], [x2, y2]]."
        ),
    )
    def mcp_segment(
        img_path: Annotated[str, "Local path to the image to segment."],
        points: Annotated[List[List[float]], "Point prompts in image coordinates, e.g. [[210, 350], [250, 220]]."],
        labels: Annotated[Optional[List[int]], "Optional point labels. Use 1 for positive and 0 for negative. Defaults to all positive."] = None,
        vis_name: Annotated[Optional[str], "Optional name used in saved visualization filename."] = "segment",
        save_vis: Annotated[bool, "Whether to save a visualization image."] = True,
    ) -> dict:
        """Segment an object in one image from point prompts."""
        mcp_log("sam2_segment", f"img_path={img_path}, points={points}, labels={labels}, vis_name={vis_name}")
        with model._lock:
            img = _load_image(img_path)
            points_np = _coerce_points(points)
            assert points_np is not None
            labels_np = _coerce_labels(labels, expected_len=len(points_np))
            bbox, vis_path = model.segment(
                img=img,
                points=points_np,
                labels=labels_np,
                vis_name=vis_name,
                save_vis=save_vis,
            )
        result = {
            "ok": True,
            "bbox": _bbox_to_list(bbox),
            "vis_path": vis_path,
        }
        mcp_log("sam2_segment", f"bbox={result['bbox']}, vis_path={vis_path}")
        return result

    @mcp.tool(
        name="track",
        description=(
            "Track/propagate the current SAM2 object state on one streaming frame. "
            "Optionally pass new point prompts for this frame. Returns bbox and frame_idx."
        ),
    )
    def mcp_track(
        img_path: Annotated[str, "Local path to the current video/frame image."],
        points: Annotated[Optional[List[List[float]]], "Optional point prompts in image coordinates for this frame."] = None,
        labels: Annotated[Optional[List[int]], "Optional point labels. Use 1 for positive and 0 for negative. Defaults to all positive when points are provided."] = None,
        save_vis: Annotated[bool, "Whether to save a visualization image."] = True,
    ) -> dict:
        """Track/propagate the current SAM2 object state on one streaming frame."""
        mcp_log("sam2_track", f"img_path={img_path}, points={points}, labels={labels}, frame_idx={model.frame_idx}")
        with model._lock:
            img = _load_image(img_path)
            points_np = _coerce_points(points)
            labels_np = _coerce_labels(labels, expected_len=None if points_np is None else len(points_np))
            bbox, vis_path, used_frame_idx = model.track(
                img=img,
                points=points_np,
                labels=labels_np,
                save_vis=save_vis,
            )
        result = {
            "ok": True,
            "frame_idx": int(used_frame_idx),
            "next_frame_idx": int(model.frame_idx),
            "bbox": _bbox_to_list(bbox),
            "vis_path": vis_path,
        }
        mcp_log("sam2_track", f"frame_idx={used_frame_idx}, bbox={result['bbox']}, vis_path={vis_path}")
        return result

    @mcp.tool(
        name="shutdown",
        description=(
            "Shut down the SAM2 MCP server process. The process exits shortly after "
            "this tool returns so the client can receive the response."
        ),
    )
    def mcp_shutdown(
        delay_seconds: Annotated[float, "Seconds to wait before terminating the server process after returning the response."] = 0.5,
        exit_code: Annotated[int, "Process exit code to use when shutting down."] = 0,
    ) -> dict:
        """Shut down the SAM2 MCP server process after returning a response."""
        mcp_log("sam2_shutdown", f"Shutdown requested: delay_seconds={delay_seconds}, exit_code={exit_code}")
        return _schedule_process_shutdown(delay_seconds=delay_seconds, exit_code=exit_code)

    return mcp


def _run_fastmcp(
    model: Sam2VideoPredictor,
    transport: str,
    host: str,
    port: int,
    block: bool = True,
):
    mcp = _create_fastmcp(model)

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
    parser = argparse.ArgumentParser(description="SAM2 stream predictor MCP server")
    parser.add_argument("--sam2-cfg", default=SAM2Config.sam2_cfg)
    parser.add_argument("--sam2-ckpt", default=SAM2Config.sam2_ckpt)
    parser.add_argument("--hydra-config", default=SAM2Config.hydra_config)
    parser.add_argument("--video-folder", default=SAM2Config.video_folder)
    parser.add_argument("--device", default=SAM2Config.device)
    parser.add_argument("--dtype", default=SAM2Config.dtype, choices=["bfloat16", "bf16", "float16", "fp16", "float32", "fp32"])
    parser.add_argument("--video-height", type=int, default=SAM2Config.video_height)
    parser.add_argument("--video-width", type=int, default=SAM2Config.video_width)
    parser.add_argument("--vis-dir", default=SAM2Config.vis_dir)
    parser.add_argument(
        "--subproj-root",
        action="append",
        default=None,
        help="Project root to prepend to sys.path. Can be provided multiple times. Defaults to the original hard-coded project roots.",
    )
    parser.add_argument("--fastmcp-transport", default="streamable-http", choices=["sse", "streamable-http", "stdio"])
    parser.add_argument("--fastmcp-host", default="127.0.0.1")
    parser.add_argument("--fastmcp-port", type=int, default=8758)
    return parser


def main():
    parser = _build_arg_parser()
    args = parser.parse_args()

    cfg = SAM2Config(
        sam2_cfg=args.sam2_cfg,
        sam2_ckpt=args.sam2_ckpt,
        video_folder=args.video_folder,
        hydra_config=args.hydra_config,
        device=args.device,
        dtype=args.dtype,
        video_height=args.video_height,
        video_width=args.video_width,
        vis_dir=args.vis_dir,
        subproj_roots=args.subproj_root,
    )

    if os.path.exists(cfg.vis_dir):
        shutil.rmtree(cfg.vis_dir)
    os.makedirs(cfg.vis_dir, exist_ok=True)

    model = Sam2VideoPredictor(cfg)

    print(
        f"FastMCP ready: transport={args.fastmcp_transport}, "
        f"host={args.fastmcp_host}, port={args.fastmcp_port}"
    )
    _run_fastmcp(
        model,
        transport=args.fastmcp_transport,
        host=args.fastmcp_host,
        port=args.fastmcp_port,
        block=True,
    )


if __name__ == "__main__":
    main()
