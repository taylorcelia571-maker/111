"""Stationary full-loop planning against an occupancy grid; no ROS dependency."""
import heapq
import math
import numpy as np
from scipy.ndimage import distance_transform_edt, map_coordinates, gaussian_filter1d
from race_line import ClosedPath, positive


class LoopPlanner:
    def __init__(self, occupancy, resolution, origin, margin=0.26):
        self.resolution = positive('resolution', resolution)
        self.margin = positive('margin', margin)
        self.origin = np.asarray(origin, dtype=float)
        grid = np.asarray(occupancy)
        if grid.ndim != 2 or not np.isfinite(self.origin).all() or self.origin.shape != (2,):
            raise ValueError('Invalid map geometry')
        # Unknown cells are obstacles. Subtract a cell diagonal for conservative interpolation.
        self.clearance = distance_transform_edt(grid == 0) * resolution - resolution * math.sqrt(2)
        self.allowed = self.clearance >= margin

    def clearance_at(self, points):
        p = np.asarray(points)
        cells = (p - self.origin) / self.resolution - 0.5
        return map_coordinates(self.clearance, [cells[:, 1], cells[:, 0]],
                               order=1, mode='constant', cval=-1)

    def segment_check(self, points):
        a = np.asarray(points)
        b = np.roll(a, -1, axis=0)
        n = max(1, int(math.ceil(np.max(np.linalg.norm(b-a, axis=1)) / (self.resolution / 2))))
        return min(float(self.clearance_at(a + (b-a)*t).min()) for t in np.linspace(0, 1, n+1))

    def edge(self, start, goal):
        def cell(p):
            c = np.floor((np.asarray(p)-self.origin)/self.resolution).astype(int)
            if not (0 <= c[0] < self.allowed.shape[1] and 0 <= c[1] < self.allowed.shape[0]):
                raise ValueError('Waypoint outside map')
            q = (int(c[1]), int(c[0]))
            if not self.allowed[q]: raise ValueError('Waypoint lacks vehicle clearance')
            return q
        start, goal = cell(start), cell(goal)
        distances = {start: 0.0}
        parent = {}
        queue = [(math.hypot(start[0]-goal[0], start[1]-goal[1]), 0.0, start)]
        moves = [(dy, dx, math.hypot(dy, dx)) for dy in (-1, 0, 1) for dx in (-1, 0, 1) if dy or dx]
        h, w = self.allowed.shape
        while queue:
            _, cost, a = heapq.heappop(queue)
            if cost != distances.get(a): continue
            if a == goal:
                result = [a]
                while result[-1] != start: result.append(parent[result[-1]])
                return (np.array(result[::-1])[:, ::-1] + 0.5)*self.resolution + self.origin
            for dy, dx, length in moves:
                b = a[0]+dy, a[1]+dx
                if not (0 <= b[0] < h and 0 <= b[1] < w) or not self.allowed[b]: continue
                if dy and dx and (not self.allowed[a[0]+dy, a[1]] or not self.allowed[a[0], a[1]+dx]): continue
                penalty = 1 + 8*math.exp(-max(0, self.clearance[b]-self.margin)/0.18)
                new = cost + length*penalty
                if new < distances.get(b, float('inf')):
                    distances[b], parent[b] = new, a
                    heapq.heappush(queue, (new+math.hypot(b[0]-goal[0], b[1]-goal[1]), new, b))
        raise ValueError('No connected collision-free route between waypoints')

    def build(self, waypoints, max_steer=0.48, wheelbase=0.26):
        anchors = np.asarray(waypoints, dtype=float)
        if anchors.ndim != 2 or anchors.shape[1] != 2 or len(anchors) < 3 or not np.isfinite(anchors).all():
            raise ValueError('Invalid ordered waypoints')
        raw = np.concatenate([self.edge(a, b)[:-1] for a,b in zip(anchors, np.roll(anchors,-1,axis=0))])
        path = ClosedPath(np.vstack([raw, raw[0]]), spacing=0.04)
        original = path.points.copy()
        # Periodic smoothing also smooths the finish-line seam. Each candidate must
        # retain clearance and remain within the same corridor as the A* reference.
        chosen = None
        limit = math.tan(max_steer)/wheelbase
        for width in np.arange(0.12, 0.71, 0.02):
            candidate = gaussian_filter1d(original, width/path.ds, axis=0, mode='wrap')
            if np.max(np.linalg.norm(candidate-original, axis=1)) > 0.40: continue
            if self.segment_check(candidate) < self.margin: continue
            trial = ClosedPath(np.vstack([candidate, candidate[0]]), spacing=0.04)
            # Check unsmoothed adjacent-point curvature as well as the tracking window.
            d = np.roll(trial.points, -1, axis=0)-trial.points
            yaw = np.unwrap(np.arctan2(d[:,1], d[:,0]))
            turns = np.arctan2(np.sin(np.roll(yaw,-1)-yaw), np.cos(np.roll(yaw,-1)-yaw))/trial.ds
            if np.max(np.abs(turns)) > limit*0.95: continue
            if any(trial.project(*p)[1] > 0.4 for p in anchors): continue
            chosen = trial  # favor smoother feasible trajectory
        if chosen is None:
            raise ValueError('Route cannot meet clearance and steering limits; do not drive')
        return chosen
