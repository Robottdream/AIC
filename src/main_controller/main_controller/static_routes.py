"""Static route estimates for ranking only; never an executable robot path."""
from collections import OrderedDict
import hashlib
import math

import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra


class StaticRouteCosts:
    """Cache a few goal distance fields, invalidated by map content/geometry.

    The 8-connected graph excludes unknown/occupied cells, inflates by 0.30 m
    and forbids diagonal corner cutting. It cannot model heading or moving
    actors, so even infinite estimates remain eligible for full-plan fallback.
    """

    def __init__(self, cache_size=4):
        self.version = None
        self.graph = None
        self.fields = OrderedDict()
        self.cache_size = cache_size
        self.field_builds = 0

    def update(self, width, height, resolution, origin, data):
        if (width < 1 or height < 1 or not math.isfinite(resolution)
                or resolution <= 0 or len(data) != width * height
                or not all(math.isfinite(v) for v in origin)):
            raise ValueError('Invalid static map geometry')
        cells = np.asarray(data, dtype=np.int8).reshape(height, width)
        version = (width, height, resolution, tuple(origin),
                   hashlib.sha256(cells.tobytes()).digest())
        if version == self.version:
            return False
        free = (cells >= 0) & (cells < 65)
        # Pad the boundary so out-of-map space is an obstacle, too.
        clearance = distance_transform_edt(np.pad(free, 1))[1:-1, 1:-1] * resolution
        free &= clearance >= .30
        ids = np.arange(width * height).reshape(height, width)
        rows, cols, weights = [], [], []
        for dy, dx in ((0, 1), (1, 0), (1, 1), (1, -1)):
            y0, y1 = max(0, -dy), height - max(0, dy)
            x0, x1 = max(0, -dx), width - max(0, dx)
            a = np.s_[y0:y1, x0:x1]
            b = np.s_[y0+dy:y1+dy, x0+dx:x1+dx]
            valid = free[a] & free[b]
            if dx and dy:
                valid &= free[y0:y1, x0+dx:x1+dx] & free[y0+dy:y1+dy, x0:x1]
            first, second = ids[a][valid], ids[b][valid]
            gap = np.minimum(clearance[a][valid], clearance[b][valid])
            # Match the planner's soft preference for 0.85 m wall clearance.
            preference = 3.0 * np.maximum(0., (.85 - gap) / .85) ** 2
            cost = resolution * math.hypot(dx, dy) * (1. + preference)
            rows.extend((first, second)); cols.extend((second, first))
            weights.extend((cost, cost))
        graph = coo_matrix((np.concatenate(weights),
                            (np.concatenate(rows), np.concatenate(cols))),
                           shape=(width * height, width * height)).tocsr()
        self.width, self.height, self.resolution = width, height, resolution
        self.origin, self.free, self.graph = tuple(origin), free, graph
        self.version = version
        self.fields.clear()
        return True

    def _index(self, point):
        if not all(math.isfinite(v) for v in point[:2]):
            return None
        x = math.floor((point[0] - self.origin[0]) / self.resolution)
        y = math.floor((point[1] - self.origin[1]) / self.resolution)
        if 0 <= x < self.width and 0 <= y < self.height and self.free[y, x]:
            return y * self.width + x
        return None

    def costs_from(self, source, targets, cache=True):
        """A single C-backed Dijkstra scores all candidates at once.

        Cache fixed area fields; transient robot starts must not evict them.
        """
        if self.graph is None:
            return None
        index = self._index(source)
        if index is None:
            return [math.inf] * len(targets)
        field = self.fields.get(index) if cache else None
        if field is None:
            field = dijkstra(self.graph, directed=True, indices=index)
            self.field_builds += 1
            if cache:
                self.fields[index] = field
                if len(self.fields) > self.cache_size:
                    self.fields.popitem(last=False)
        if cache:
            self.fields.move_to_end(index)
        result = []
        for point in targets:
            target = self._index(point)
            result.append(float(field[target]) if target is not None else math.inf)
        return result
