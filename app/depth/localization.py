"""Mask depth localization matching the Robot client's public world coordinates."""
import cv2
import numpy as np


class TargetNotLocalizable(ValueError):
    pass


def decode_localization(data, shape):
    value = data.get('mask')
    if (not isinstance(value, dict) or value.get('encoding') != 'rle'
            or value.get('order') != 'C' or value.get('size') != list(shape)):
        raise ValueError('mask must be original-image row-major RLE')
    counts = value.get('counts')
    if (not isinstance(counts, list) or not counts or len(counts) > int(np.prod(shape)) + 1
            or any(type(n) is not int or n < 0 for n in counts) or sum(counts) != int(np.prod(shape))):
        raise ValueError('invalid mask RLE counts')
    mask = np.repeat(np.arange(len(counts)) % 2 == 1, counts).reshape(shape)
    try:
        transform = np.asarray(data.get('world_from_camera_cm'), dtype=float)
        distortion = data.get('distortion')
        distortion = None if distortion is None else np.asarray(distortion, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError('invalid camera geometry') from exc
    if (transform.shape != (4, 4) or not np.isfinite(transform).all()
            or not np.allclose(transform[3], [0, 0, 0, 1])
            or not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-4)
            or not np.isclose(np.linalg.det(transform[:3, :3]), 1., atol=1e-4)):
        raise ValueError('world_from_camera_cm must be a rigid 4x4 camera-to-world transform')
    if distortion is not None and (distortion.ndim != 1 or len(distortion) not in (4, 5, 8, 12, 14)
                                   or not np.isfinite(distortion).all()):
        raise ValueError('invalid distortion coefficients')
    min_pixels = data.get('min_depth_pixels', 16)
    max_mad = data.get('max_relative_depth_mad', .3)
    if type(min_pixels) is not int or not 1 <= min_pixels <= int(np.prod(shape)):
        raise ValueError('invalid min_depth_pixels')
    if type(max_mad) not in (float, int) or not np.isfinite(max_mad) or max_mad <= 0:
        raise ValueError('invalid max_relative_depth_mad')
    return mask, transform, distortion, min_pixels, max_mad


def locate(depth, intrinsics, mask, transform, distortion, min_pixels, max_relative_mad):
    ys, xs = np.nonzero(mask)
    if len(xs) < min_pixels:
        raise TargetNotLocalizable('insufficient target mask pixels')
    pixels = np.column_stack((xs, ys)).astype(np.float64)
    if distortion is not None:
        pixels = cv2.undistortPoints(pixels.reshape(-1, 1, 2), intrinsics,
                                     distortion, P=intrinsics).reshape(-1, 2)
    depths = depth[ys, xs]
    valid = np.isfinite(depths) & (depths > 0) & np.isfinite(pixels).all(axis=1)
    pixels, depths = pixels[valid], depths[valid]
    if len(depths) < min_pixels:
        raise TargetNotLocalizable('insufficient valid target depth pixels')
    median = float(np.median(depths))
    mad = float(np.median(np.abs(depths - median)))
    if mad / median > max_relative_mad:
        raise TargetNotLocalizable('target depth is too dispersed')
    keep = np.abs(depths - median) <= max(3 * mad, median * .05)
    if np.count_nonzero(keep) < min_pixels:
        raise TargetNotLocalizable('insufficient target depth inliers')
    rays = np.linalg.solve(intrinsics, np.column_stack((pixels[keep], np.ones(np.count_nonzero(keep)))).T)
    camera_cm = rays * depths[keep] * 100
    world = transform[:3, :3] @ camera_cm + transform[:3, 3:4]
    point = np.median(world, axis=1) * [1, -1, 1]
    if not np.isfinite(point).all():
        raise TargetNotLocalizable('nonfinite depth target position')
    return dict(target_position_cm=point.tolist(), valid_depth_pixels=len(depths),
                inlier_pixels=int(keep.sum()), median_depth_m=median, relative_depth_mad=mad / median)
