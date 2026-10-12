"""Closed-path pure pursuit and cyclic braking envelope; no ROS dependency."""
import math
import hashlib
import numpy as np


def positive(name, value):
    if not math.isfinite(value) or value <= 0:
        raise ValueError(name + ' must be finite and positive')
    return float(value)


class ClosedPath:
    def fingerprint(self):
        """Identify reference geometry independently of mutable speed limits."""
        return hashlib.sha256(np.round(self.points, 4).astype('<f8').tobytes()).hexdigest()

    def __init__(self, points, spacing=0.08, closure_limit=0.6, max_gap=0.6):
        spacing = positive('spacing', spacing)
        positive('closure_limit', closure_limit)
        positive('max_gap', max_gap)
        p = np.asarray(points, dtype=float)
        if p.ndim != 2 or p.shape[1] != 2 or len(p) < 12 or not np.isfinite(p).all():
            raise ValueError('Need at least 12 finite [x,y] points from a full closed lap')
        # Remove stationary duplicates without changing the recorded route.
        p = p[np.r_[True, np.linalg.norm(np.diff(p, axis=0), axis=1) > 1e-5]]
        if len(p) < 12 or np.linalg.norm(p[-1] - p[0]) > closure_limit:
            raise ValueError('Path is not closed; record a complete valid lap')
        if np.linalg.norm(p[-1] - p[0]) < 1e-5:
            p = p[:-1]
        segments = np.roll(p, -1, axis=0) - p
        lengths = np.linalg.norm(segments, axis=1)
        if np.any(lengths > max_gap) or np.any(lengths < 1e-6):
            raise ValueError('Path contains a localization jump or missing samples')
        self.length = float(lengths.sum())
        if self.length < 10:
            raise ValueError('Path is too short to represent the race course')
        if self.length / spacing > 30000:
            raise ValueError('Path has too many samples; check recording')
        knots = np.r_[0, np.cumsum(lengths)]
        closed = np.vstack([p, p[0]])
        self.n = max(12, int(math.ceil(self.length / spacing)))
        self.ds = self.length / self.n
        self.points = np.column_stack([np.interp(np.arange(self.n) * self.ds, knots, closed[:, j])
                                      for j in (0, 1)])
        # Curvature estimated over a physical window, not raw noisy pose steps.
        w = max(1, int(round(0.3 / self.ds)))
        a = self.points - np.roll(self.points, w, axis=0)
        b = np.roll(self.points, -w, axis=0) - self.points
        c = a + b
        area2 = a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]
        denominator = np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1) * np.linalg.norm(c, axis=1)
        self.curvature = 2 * area2 / np.maximum(denominator, 1e-9)
        self.speeds = None

    def sample(self, distance):
        index = (distance % self.length) / self.ds
        i = int(index) % self.n
        f = index - int(index)
        return self.points[i] * (1 - f) + self.points[(i + 1) % self.n] * f

    def project(self, x, y, begin=None, end=None):
        # Restrict projection to a forward arc to prevent jumps at nearby lanes.
        if begin is None:
            ids = np.arange(self.n)
        else:
            ids = np.arange(int(math.floor(begin / self.ds)), int(math.ceil(end / self.ds)) + 1)
        a = self.points[ids % self.n]
        b = self.points[(ids + 1) % self.n] - a
        q = np.array([x, y]) - a
        t = np.clip(np.sum(q * b, axis=1) / np.maximum(np.sum(b * b, axis=1), 1e-9), 0, 1)
        projected = a + t[:, None] * b
        d = np.linalg.norm(projected - [x, y], axis=1)
        i = int(np.argmin(d))
        s = float((ids[i] + t[i]) * self.ds)
        if begin is not None:
            s = max(begin, min(end, s))
        return s, float(d[i])

    def plan_speed(self, max_speed=3.0, lateral_accel=3.0, acceleration=2.5, braking=3.5):
        for name, value in locals().copy().items():
            if name != 'self': positive(name, value)
        curve = np.abs(self.curvature)
        # Protect neighboring samples; do not hide isolated tight corners.
        curve = np.maximum.reduce([np.roll(curve, j) for j in range(-2, 3)])
        speed = np.minimum(max_speed, np.sqrt(lateral_accel / np.maximum(curve, 1e-6)))
        # Two passes enforce cyclic constraints across the start/finish seam.
        for _ in range(2):
            for i in range(self.n - 1, -1, -1):
                speed[i] = min(speed[i], math.sqrt(speed[(i + 1) % self.n] ** 2 + 2 * braking * self.ds))
            for i in range(self.n):
                speed[i] = min(speed[i], math.sqrt(speed[(i - 1) % self.n] ** 2 + 2 * acceleration * self.ds))
        self.speeds = speed
        return speed.copy()


class LineFollower:
    def __init__(self, path, max_speed=3.0, lateral_accel=3.0, acceleration=2.5,
                 braking=3.5, wheelbase=0.265, max_steer=0.48, max_error=0.6):
        for name, value in locals().copy().items():
            if name not in ('self', 'path'): positive(name, value)
        if max_steer >= math.pi / 2: raise ValueError('Invalid steering limit')
        self.path = path
        path.plan_speed(max_speed, lateral_accel, acceleration, braking)
        self.max_speed, self.lateral_accel = max_speed, lateral_accel
        self.acceleration, self.braking = acceleration, braking
        self.wheelbase, self.max_steer, self.max_error = wheelbase, max_steer, max_error
        self.progress = None
        self.origin = None
        self.speed = 0.0
        self.error = 0.0

    def start(self, x, y, yaw):
        if not all(math.isfinite(v) for v in (x, y, yaw)): raise ValueError('Invalid pose')
        s, error = self.path.project(x, y)
        a, b = self.path.sample(s), self.path.sample(s + 0.4)
        heading = math.atan2(b[1] - a[1], b[0] - a[0])
        difference = math.atan2(math.sin(heading - yaw), math.cos(heading - yaw))
        if error > self.max_error or abs(difference) > math.pi / 3:
            raise ValueError('Start is too far from path or heading is reversed')
        self.progress = self.origin = s

    def stop(self):
        self.speed = 0.0

    def step(self, x, y, yaw, measured_speed, dt):
        if not all(math.isfinite(v) for v in (x, y, yaw, measured_speed, dt)) or not 0 < dt <= 0.2:
            self.stop()
            raise ValueError('Invalid pose/speed or stale control cycle')
        if self.progress is None: self.start(x, y, yaw)
        s, self.error = self.path.project(x, y, self.progress - 0.15,
                                         self.progress + self.max_speed * dt * 2 + 0.25)
        if self.error > self.max_error:
            self.stop()
            raise ValueError('Car departed recorded route')
        self.progress = max(self.progress, s)
        # Preview shrinks in corners; longer on straights reduces hunting.
        i = int(self.progress / self.path.ds) % self.path.n
        planned = float(self.path.speeds[i])
        lookahead = max(0.45, min(1.25, 0.4 + 0.25 * max(0, measured_speed)))
        preview_curve = max(abs(self.path.curvature[(i + j) % self.path.n])
                            for j in range(int(lookahead / self.path.ds) + 1))
        lookahead = min(lookahead, max(0.45, 1.1 / (1 + preview_curve)))
        target = self.path.sample(self.progress + lookahead)
        dx, dy = target - [x, y]
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        curvature = 2 * local_y / max(dx * dx + dy * dy, 1e-6)
        steer = max(-self.max_steer, min(self.max_steer, math.atan(self.wheelbase * curvature)))
        desired = min(planned, math.sqrt(self.lateral_accel / max(abs(curvature), 1e-6)))
        # Reduce pace while off the reference route instead of cutting harder.
        desired *= max(0.35, 1 - (self.error / self.max_error) ** 2)
        self.speed = min(desired, self.speed + self.acceleration * dt)
        return self.speed, steer


def clearance_speed(ranges, angle_min, increment, range_min, range_max,
                    speed, steer, wheelbase=0.265, half_width=0.20,
                    braking=3.5, reaction=0.15, laser_x=0.0, laser_y=0.0, laser_yaw=0.0):
    """Laser points in the swept circular corridor; no obstacle avoidance."""
    if not math.isfinite(increment) or increment <= 0 or len(ranges) < 3:
        return 0.0
    r = np.asarray(ranges, dtype=float)
    angles = angle_min + np.arange(len(r)) * increment + laser_yaw
    angles = np.arctan2(np.sin(angles), np.cos(angles))
    forward = np.abs(angles) <= math.pi / 2
    valid = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    # Too many invalid beams (except +inf=no return) cannot prove clearance.
    known = valid | np.isposinf(r)
    if not forward.any() or np.mean(known[forward]) < 0.9: return 0.0
    x, y = laser_x + r[valid & forward] * np.cos(angles[valid & forward]), laser_y + r[valid & forward] * np.sin(angles[valid & forward])
    curvature = math.tan(steer) / wheelbase
    horizon = speed * reaction + speed * speed / (2 * braking) + 0.45
    arc = np.linspace(0, max(0.6, horizon), 50)
    if abs(curvature) < 1e-5:
        cx, cy = arc, np.zeros_like(arc)
    else:
        cx, cy = np.sin(curvature * arc) / curvature, (1 - np.cos(curvature * arc)) / curvature
    distance = np.hypot(x[:, None] - cx, y[:, None] - cy)
    hits = np.where(np.any(distance < half_width, axis=0))[0]
    if len(hits) == 0: return float('inf')
    free = max(0.0, float(arc[hits[0]]) - 0.30)
    # Solve v*reaction + v²/(2b) <= free.
    return max(0.0, math.sqrt((braking * reaction) ** 2 + 2 * braking * free) - braking * reaction)
