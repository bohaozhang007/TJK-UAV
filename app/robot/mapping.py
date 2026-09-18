"""Platform-independent operations on a bounded occupancy grid and camera geometry."""
import numpy as np


class GridMap:
    def __init__(self, data):
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

    def free(self, public_pose):
        xyz = np.array([public_pose['x'], -public_pose['y'], public_pose['z']]) / 100
        index = np.floor(xyz / self.resolution).astype(int)
        return bool(np.all(index >= self.lower) and np.all(index <= self.upper)
                    and self.ground < xyz[2] < self.ceiling and tuple(index) not in self.inflated)

    def locate(self, obs, mask, *, min_voxels=2, depth_gap_m=.3):
        if mask.shape != obs.rgb.shape[:2] or mask.dtype != np.bool_:
            raise ValueError('mask and exposure image do not match')
        world = (self.occupied.astype(float) + .5) * self.resolution * 100
        t = obs.world_from_camera_cm
        camera = (world - t[:3, 3]) @ t[:3, :3]
        visible = camera[:, 2] > 1e-3
        world, camera = world[visible], camera[visible]
        pixels = camera @ obs.intrinsics.T
        uv = np.rint(pixels[:, :2] / pixels[:, 2:3]).astype(int)
        h, w = mask.shape
        inside = (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)
        world, camera, uv = world[inside], camera[inside], uv[inside]
        selected = mask[uv[:, 1], uv[:, 0]]
        world, camera, uv = world[selected], camera[selected], uv[selected]
        if len(world) < min_voxels:
            raise ValueError('insufficient occupied target voxels in mask')
        # Nearest visible surface at each pixel, then separate foreground/background depths.
        order = np.argsort(camera[:, 2], kind='stable')
        _, first = np.unique(uv[order, 1] * w + uv[order, 0], return_index=True)
        order = order[np.sort(first)]
        world, camera = world[order], camera[order]
        groups = np.split(np.arange(len(world)), np.flatnonzero(np.diff(camera[:, 2]) > depth_gap_m*100) + 1)
        groups = [group for group in groups if len(group) >= min_voxels]
        if not groups:
            raise ValueError('no supported target surface in mask')
        # Use the foremost supported depth group, not a wall behind the object.
        candidates = world[groups[0]]
        center = np.median(candidates, axis=0)
        point = candidates[np.argmin(np.linalg.norm(candidates-center, axis=1))].copy()
        point[1] *= -1
        return point.tolist()  # Exactly one actual uninflated voxel centre.
