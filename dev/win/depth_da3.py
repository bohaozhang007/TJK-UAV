import pickle
from contextlib import redirect_stdout
import os
from pathlib import Path
import sys

import cv2
import numpy as np
from scipy.spatial.transform import Rotation


MODEL_ROOT = Path(__file__).resolve().parents[3] / "depth-anything-3"
sys.path.insert(0, str(MODEL_ROOT / "src"))
os.environ.setdefault("XFORMERS_FORCE_DISABLE_TRITON", "1")
with redirect_stdout(sys.stderr):
    from depth_anything_3.api import DepthAnything3


CHECKPOINT = MODEL_ROOT / "checkpoints/DA3NESTED-GIANT-LARGE"

# Optical right/down/forward -> body forward/left/up; no mounting offset or gimbal rotation.
BODY_FROM_OPTICAL = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
MIN_MASK_PIXELS = 16
MAX_RELATIVE_DEPTH_MAD = 0.3


def world_from_camera(pose):
    position = np.asarray(pose["position_m"], dtype=float)
    quaternion = np.asarray(pose["quaternion_xyzw"], dtype=float)
    if (
        position.shape != (3,)
        or quaternion.shape != (4,)
        or not np.isfinite(position).all()
        or not np.isfinite(quaternion).all()
        or abs(np.linalg.norm(quaternion) - 1.0) > 0.02
    ):
        raise ValueError("Pose must contain a finite position and unit xyzw quaternion")
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_quat(quaternion).as_matrix() @ BODY_FROM_OPTICAL
    transform[:3, 3] = position
    return transform


def mask_points(
    depth,
    mask,
    intrinsics,
):
    """Return robust visible-surface points in optical coordinates, in metres."""
    ys, xs = np.nonzero(
        mask
        & np.isfinite(depth)
        & (depth > 0)
    )
    if len(xs) < MIN_MASK_PIXELS:
        raise ValueError("Insufficient valid target depth pixels")
    z = depth[ys, xs]
    median = np.median(z)
    mad = np.median(np.abs(z - median))
    if mad / median > MAX_RELATIVE_DEPTH_MAD:
        raise ValueError("Target depth is too dispersed")
    keep = np.abs(z - median) <= max(3 * mad, median * 0.05)
    if np.count_nonzero(keep) < MIN_MASK_PIXELS:
        raise ValueError("Insufficient target depth inliers")
    pixels = np.column_stack((xs[keep], ys[keep], np.ones(keep.sum())))
    rays = np.linalg.solve(intrinsics, pixels.T).T
    return rays * z[keep, None]


class Da3Depth:
    def __init__(self):
        self.model = DepthAnything3.from_pretrained(str(CHECKPOINT)).to("cuda").eval()

    def estimate(self, img):
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        prediction = self.model.inference([rgb])
        if not prediction.is_metric:
            raise RuntimeError("DA3 did not return metric depth")
        depth = np.asarray(prediction.depth[0], dtype=np.float32)
        intrinsics = np.array(prediction.intrinsics[0], dtype=float)
        if (
            depth.ndim != 2
            or intrinsics.shape != (3, 3)
            or not np.isfinite(intrinsics).all()
            or intrinsics[0, 0] <= 0
            or intrinsics[1, 1] <= 0
        ):
            raise RuntimeError("DA3 returned invalid depth or intrinsics")

        # The default DA3 resize has no crop; scale depth and predicted K back together.
        h, w = img.shape[:2]
        intrinsics[0] *= w / depth.shape[1]
        intrinsics[1] *= h / depth.shape[0]
        depth = cv2.resize(
            depth,
            (w, h),
            interpolation=cv2.INTER_LINEAR,
        )
        return depth, intrinsics

    def locate(
        self,
        img,
        masks,
        pose,
    ):
        transform = world_from_camera(pose)
        depth, intrinsics = self.estimate(img)
        results = []
        for mask in masks:
            if mask.shape != img.shape[:2]:
                raise ValueError("Target mask must match the current image dimensions")
            result = {"position_camera_m": None, "position_world_m": None,
                      "point_count": 0, "localization_error": None}
            try:
                points = mask_points(
                    depth,
                    mask,
                    intrinsics,
                )
                camera_position = np.median(points, axis=0)
                world_position = transform[:3, :3] @ camera_position + transform[:3, 3]
                result["position_camera_m"] = camera_position.tolist()
                result["position_world_m"] = world_position.tolist()
                result["point_count"] = len(points)
            except Exception as exc:
                result["localization_error"] = str(exc)
            results.append(result)
        return results


def main():
    # Reserve stdout for binary replies; model logs go to stderr.
    output = sys.stdout.buffer
    sys.stdout = sys.stderr

    def reply(result, error=None):
        pickle.dump((result, error), output)
        output.flush()

    try:
        args = pickle.load(sys.stdin.buffer)
        img = np.zeros((1080, 1920, 3), dtype=np.uint8)
        print(f"Loading da3 with {sys.executable}...", flush=True)
        model = Da3Depth(*args)
        print("Warming up da3 at 1920 x 1080...", flush=True)
        model.estimate(img)
    except Exception as exc:
        reply(None, str(exc))
        return
    reply(None)

    while True:
        try:
            args = pickle.load(sys.stdin.buffer)
        except EOFError:
            return
        try:
            result = model.locate(*args)
        except Exception as exc:
            reply(None, str(exc))
        else:
            reply(result)


if __name__ == "__main__":
    main()
