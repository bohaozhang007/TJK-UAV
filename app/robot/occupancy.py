"""Lossless XYZ-major occupancy flags; Z varies fastest, low bits first."""
import base64
import math

import numpy as np


class VoxelOccupancy:
    def __init__(self, data):
        self.lower = np.asarray(data['lower'], dtype=np.int64)
        self.upper = np.asarray(data['upper'], dtype=np.int64)
        if (self.lower.shape != (3,) or self.upper.shape != (3,)
                or np.any(self.upper < self.lower)):
            raise ValueError('invalid occupancy bounds')
        shape = tuple(int(b)-int(a)+1 for a, b in zip(self.lower, self.upper))
        count = math.prod(shape)
        if count > 8000000:
            raise ValueError('occupancy exceeds voxel budget')
        encoding = data.get('encoding')
        if encoding == 'voxel-flags-2bit-v1':
            payload = data['cells_base64']
            byte_count = (count+3)//4
            if not isinstance(payload, str) or len(payload) != ((byte_count+2)//3)*4:
                raise ValueError('invalid occupancy payload length')
            raw = base64.b64decode(payload, validate=True)
            if len(raw) != byte_count:
                raise ValueError('invalid occupancy byte count')
            packed = np.frombuffer(raw, dtype=np.uint8)
            flags = ((packed[:, None] >> np.array([0, 2, 4, 6], dtype=np.uint8)) & 3).reshape(-1)
            if np.any(flags[count:]):
                raise ValueError('nonzero occupancy padding')
            self.flags = flags[:count].reshape(shape)
        elif encoding is None:
            # Read older planner responses during deployment transitions.
            self.flags = np.zeros(shape, dtype=np.uint8)
            for key, flag in (('occupied', 1), ('inflated', 2)):
                indices = np.asarray(data[key], dtype=np.int64).reshape(-1, 3)
                if np.any(indices < self.lower) or np.any(indices > self.upper):
                    raise ValueError('occupancy coordinate outside bounds')
                local = indices-self.lower
                self.flags[tuple(local.T)] |= flag
        else:
            raise ValueError('unsupported occupancy encoding: '+str(encoding))

    def occupied_indices(self):
        return np.argwhere(self.flags & 1)+self.lower

    def inflated_at(self, indices):
        indices = np.asarray(indices, dtype=np.int64).reshape(-1, 3)
        inside = np.all((indices >= self.lower) & (indices <= self.upper), axis=1)
        result = np.zeros(len(indices), dtype=bool)
        local = indices[inside]-self.lower
        result[inside] = (self.flags[tuple(local.T)] & 2) != 0
        return result

    def __contains__(self, index):
        index = np.asarray(index, dtype=np.int64)
        if np.any(index < self.lower) or np.any(index > self.upper):
            return False
        return bool(self.flags[tuple(index-self.lower)] & 2)
