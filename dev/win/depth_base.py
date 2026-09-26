import pickle
import sys

import numpy as np
from scipy.spatial.transform import Rotation


WARMUP_SHAPE = (1080, 1920, 3)

# Optical right/down/forward -> body forward/left/up; no mounting offset or gimbal rotation.
BODY_FROM_OPTICAL = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
MIN_MASK_PIXELS = 16
MAX_RELATIVE_DEPTH_MAD = 0.3


class DepthEstimator:
    """Shared localization and warmup for models implementing estimate(img)."""
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

    def warmup(self):
        img = np.zeros(WARMUP_SHAPE, dtype=np.uint8)
        self.estimate(img)


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


def run_depth(model_class, name, output):
    try:
        pickle.load(sys.stdin.buffer)
        print(f"[{name}] Loading model...", flush=True)
        model = model_class()
        print(f"[{name}] Model loaded.", flush=True)
        print(f"[{name}] Warming up at 1920 x 1080...", flush=True)
        model.warmup()
        print(f"[{name}] Warmup complete.", flush=True)
    except Exception as exc:
        pickle.dump((None, str(exc)), output)
        output.flush()
        return
    pickle.dump((None, None), output)
    output.flush()

    while True:
        try:
            img, masks, pose = pickle.load(sys.stdin.buffer)
        except EOFError:
            return
        try:
            result = model.locate(img, masks, pose)
        except Exception as exc:
            pickle.dump((None, str(exc)), output)
            output.flush()
        else:
            pickle.dump((result, None), output)
            output.flush()

