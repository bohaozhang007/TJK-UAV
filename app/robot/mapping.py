"""Platform-independent operations on a bounded occupancy grid and camera geometry."""
import numpy as np


class TargetSurfaceUnavailable(ValueError):
    pass


def project_camera(camera, intrinsics, distortion=None):
    if not len(camera):
        return np.empty((0, 2), float)
    if distortion is not None:
        import cv2
        return cv2.projectPoints(np.asarray(camera, float), np.zeros(3), np.zeros(3),
                                 intrinsics, distortion)[0].reshape(-1, 2)
    pixels = camera @ intrinsics.T
    return pixels[:, :2] / pixels[:, 2:3]


class GridMap:
    def __init__(self, data):
        self.metadata = {k:data[k] for k in ('version', 'stamp_s', 'age_s', 'resolution_m', 'lower', 'upper') if k in data}
        self.resolution = float(data['resolution_m'])
        self.lower = np.asarray(data['lower'], dtype=int)
        self.upper = np.asarray(data['upper'], dtype=int)
        self.occupied = np.asarray(data['occupied'], dtype=int).reshape(-1, 3)
        self.inflated = {tuple(v) for v in data['inflated']}
        self.ground = float(data['ground_m'])
        self.ceiling = float(data['ceiling_m'])
        if (not np.isfinite(self.resolution) or self.resolution <= 0
                or self.lower.shape != (3,) or self.upper.shape != (3,)
                or np.any(self.upper < self.lower)):
            raise ValueError('invalid grid dimensions')

    def free(self, public_pose, clearance_cm=0.):
        xyz = np.array([public_pose['x'], -public_pose['y'], public_pose['z']]) / 100
        index = np.floor(xyz / self.resolution).astype(int)
        radius = clearance_cm / 100
        if not np.isfinite(radius) or radius < 0:
            raise ValueError('clearance must be finite and nonnegative')
        if radius == 0:
            return bool(np.all(index >= self.lower) and np.all(index <= self.upper)
                        and self.ground < xyz[2] < self.ceiling and tuple(index) not in self.inflated)
        if (np.any(xyz-radius < self.lower*self.resolution)
                or np.any(xyz+radius >= (self.upper+1)*self.resolution)
                or xyz[2]-radius <= self.ground or xyz[2]+radius >= self.ceiling):
            return False
        lower = np.floor((xyz-radius)/self.resolution).astype(int)-1
        upper = np.floor((xyz+radius)/self.resolution).astype(int)
        for offset in np.ndindex(tuple(upper-lower+1)):
            cell = lower + offset
            if tuple(cell) not in self.inflated:
                continue
            # Distance to the voxel box, including contact at the sphere boundary.
            delta = np.maximum(np.maximum(cell*self.resolution-xyz,
                                         xyz-(cell+1)*self.resolution), 0.)
            if np.dot(delta, delta) <= radius*radius + 1e-12:
                return False
        return True

    def locate(self, obs, mask, *, min_voxels=2, depth_gap_m=.3, diagnostics=None):
        if mask.shape != obs.rgb.shape[:2] or mask.dtype != np.bool_:
            raise ValueError('mask and exposure image do not match')
        world = (self.occupied.astype(float) + .5) * self.resolution * 100
        t = obs.world_from_camera_cm
        camera = (world - t[:3, 3]) @ t[:3, :3]
        visible = camera[:, 2] > 1e-3
        if diagnostics is not None:
            diagnostics.update(map=self.metadata, occupied_voxels=len(world),
                camera_front_voxels=int(visible.sum()), mask_pixels=int(mask.sum()),
                min_voxels=min_voxels, depth_gap_m=depth_gap_m)
        world, camera = world[visible], camera[visible]
        uv = np.rint(project_camera(camera, obs.intrinsics, obs.distortion))
        h, w = mask.shape
        inside = np.isfinite(uv).all(axis=1) & (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)
        world, camera, uv = world[inside], camera[inside], uv[inside].astype(int)
        selected = mask[uv[:, 1], uv[:, 0]]
        if diagnostics is not None:
            diagnostics.update(image_voxels=len(uv), mask_hit_voxels=int(selected.sum()),
                uv=uv.copy(), depth_m=camera[:, 2].copy()/100, hits=selected.copy())
        world, camera, uv = world[selected], camera[selected], uv[selected]
        if len(world) < min_voxels:
            raise TargetSurfaceUnavailable(
                f'insufficient occupied target voxels in mask: found={len(world)}, required={min_voxels}')
        # Nearest visible surface at each pixel, then separate foreground/background depths.
        order = np.argsort(camera[:, 2], kind='stable')
        _, first = np.unique(uv[order, 1] * w + uv[order, 0], return_index=True)
        order = order[np.sort(first)]
        world, camera = world[order], camera[order]
        groups = np.split(np.arange(len(world)), np.flatnonzero(np.diff(camera[:, 2]) > depth_gap_m*100) + 1)
        groups = [group for group in groups if len(group) >= min_voxels]
        if diagnostics is not None:
            diagnostics.update(unique_mask_pixels=len(world), supported_depth_groups=[
                dict(voxels=len(g), near_m=float(camera[g[0], 2]/100), far_m=float(camera[g[-1], 2]/100))
                for g in groups])
        if not groups:
            raise TargetSurfaceUnavailable('no supported target surface in mask')
        # Use the foremost supported depth group, not a wall behind the object.
        candidates = world[groups[0]]
        center = np.median(candidates, axis=0)
        point = candidates[np.argmin(np.linalg.norm(candidates-center, axis=1))].copy()
        point[1] *= -1
        if diagnostics is not None:
            diagnostics['target_position_cm'] = point.tolist()
        return point.tolist()  # Exactly one actual uninflated voxel centre.
