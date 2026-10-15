"""Immutable cyclic reference cache, available before lap one; no ROS inputs.

This caches geometry and nominal speed queries, never poses, commands or safety
decisions. Live feedback and every existing protection remain mandatory.
"""
import copy
import hashlib
import math
import numpy as np


class CyclicTrackMemory:
    def __init__(self, path):
        if path.speeds is None or not np.isfinite(path.speeds).all() or np.any(path.speeds <= 0):
            raise ValueError('Track memory needs a complete positive speed profile')
        if not np.isfinite(path.points).all() or not np.isfinite(path.curvature).all():
            raise ValueError('Track memory needs finite geometry')
        self.path = path
        self.points, self.curvature, self.speeds = path.points, path.curvature, path.speeds
        self.n, self.ds, self.length = path.n, path.ds, path.length
        for array in (self.points, self.curvature, self.speeds):
            array.setflags(write=False)
        self.segments = np.roll(self.points, -1, axis=0) - self.points
        self.denominator = np.maximum(np.sum(self.segments * self.segments, axis=1), 1e-9)
        self.speed_min = self._table(self.speeds, np.minimum)
        self.curve_max = self._table(np.abs(self.curvature), np.maximum)
        self.geometry_sha256 = path.fingerprint()
        self.speed_sha256 = hashlib.sha256(self.speeds.astype('<f8').tobytes()).hexdigest()

    @staticmethod
    def _table(values, reduction):
        tables = [values]
        span = 1
        while span * 2 <= len(values):
            tables.append(reduction(tables[-1], np.roll(tables[-1], -span)))
            span *= 2
        for array in tables:
            array.setflags(write=False)
        return tables

    def __deepcopy__(self, memo):
        result = type(self)(copy.deepcopy(self.path, memo))
        memo[id(self)] = result
        return result

    def validate(self):
        if (self.path.points is not self.points or self.path.curvature is not self.curvature or
                self.path.speeds is not self.speeds or self.path.n != self.n or
                self.path.ds != self.ds or self.path.length != self.length or
                any(a.flags.writeable for a in (self.points, self.curvature, self.speeds))):
            raise ValueError('Track memory changed; rebuild while stationary')

    def _range(self, tables, index, count, reduction):
        self.validate()
        if not isinstance(count, (int, np.integer)) or not 0 <= count < self.n:
            raise ValueError('Invalid cyclic track-memory query')
        size = int(count) + 1
        level = size.bit_length() - 1
        span = 1 << level
        return float(reduction(tables[level][index % self.n],
                               tables[level][(index + size - span) % self.n]))

    def minimum_speed(self, index, count):
        return self._range(self.speed_min, index, count, min)

    def maximum_curvature(self, index, count):
        return self._range(self.curve_max, index, count, max)

    def project(self, x, y, begin, end):
        self.validate()
        # Preserve the original projection window, clipped geometry and tie order.
        ids = np.arange(int(math.floor(begin / self.ds)), int(math.ceil(end / self.ds)) + 1)
        ids = ids[(ids * self.ds <= end) & ((ids + 1) * self.ds >= begin)]
        a = self.points[ids % self.n]
        b = self.segments[ids % self.n]
        q = np.array([x, y]) - a
        t = np.clip(np.sum(q * b, axis=1) / self.denominator[ids % self.n], 0, 1)
        lower = np.clip(begin / self.ds - ids, 0, 1)
        upper = np.clip(end / self.ds - ids, 0, 1)
        t = np.clip(t, lower, upper)
        projected = a + t[:, None] * b
        d = np.linalg.norm(projected - [x, y], axis=1)
        best = int(np.argmin(d))
        s = float((ids[best] + t[best]) * self.ds)
        return max(begin, min(end, s)), float(d[best])

    def report(self):
        return dict(enabled=True, warmup_laps=0, geometry_sha256=self.geometry_sha256,
                    speed_sha256=self.speed_sha256, samples=self.n,
                    cached='fixed projection geometry and cyclic speed/curvature range queries',
                    replay_commands=False, live_feedback_required=True)
