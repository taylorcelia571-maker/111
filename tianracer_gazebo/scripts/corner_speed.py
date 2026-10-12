"""Speed-only guard with curvature preview; steering and planner stops preserved."""
import math
import numpy as np


class CornerSpeed:
    def __init__(self, straight_speed=2.0, corner_speed=1.8, acceleration=1.8,
                 braking=1.5, reaction_time=0.25, wheelbase=0.26, acceleration_ramp=False):
        values = (straight_speed, corner_speed, acceleration, braking, reaction_time, wheelbase)
        if not all(math.isfinite(v) and v > 0 for v in values) or corner_speed > straight_speed:
            raise ValueError('Invalid straight/corner speed or motion limits')
        self.straight_speed, self.corner_speed = straight_speed, corner_speed
        self.acceleration, self.braking = acceleration, braking
        self.reaction_time, self.wheelbase = reaction_time, wheelbase
        self.acceleration_ramp = acceleration_ramp
        self.speed = 0.0
        self.last_limit = 0.0
        self.reason = 'idle'

    def curve_limit(self, curvature):
        angle = math.atan(self.wheelbase * abs(curvature))
        blend = min(1.0, max(0.0, (angle - 0.06) / 0.10))
        blend = blend * blend * (3 - 2 * blend)
        speed = self.straight_speed - (self.straight_speed - self.corner_speed) * blend
        # Conservative extrapolation beyond the baseline 0.5m minimum radius.
        lateral_budget = self.corner_speed ** 2 / 0.5
        return min(speed, math.sqrt(lateral_budget / max(abs(curvature), 1e-6)))

    def step(self, requested_speed, steer, measured_steer, measured_speed, preview, dt):
        if not all(math.isfinite(v) for v in (requested_speed, steer, measured_speed, dt)) or dt <= 0:
            self.speed = 0.0
            return 0.0
        dt = min(dt, 0.1)
        if requested_speed == 0:
            self.speed, self.reason = 0.0, 'planner_stop'
            return 0.0
        if requested_speed*measured_speed < -0.0025:
            self.speed, self.reason = 0.0, 'direction_brake'
            return 0.0
        curvature = abs(math.tan(steer)/self.wheelbase)
        if measured_steer is not None:
            if not math.isfinite(measured_steer):
                self.speed = 0.0
                return 0.0
            curvature = max(curvature, abs(math.tan(measured_steer)/self.wheelbase))
        limit = self.curve_limit(curvature)
        self.reason = 'current_turn' if limit < self.straight_speed else 'straight'
        # Missing path or steering feedback cannot establish a fast straight.
        if preview is None or measured_steer is None:
            if limit > self.corner_speed: self.reason = 'feedback_fallback'
            limit = min(limit, self.corner_speed)
        else:
            reaction_distance = max(abs(measured_speed), abs(self.speed))*self.reaction_time
            for distance, curve in preview:
                if not math.isfinite(distance) or not math.isfinite(curve) or distance < 0: continue
                allowed = math.sqrt(self.curve_limit(curve)**2 +
                                    2*self.braking*max(0, distance-reaction_distance))
                if allowed < limit: limit, self.reason = allowed, 'preview_turn'
        self.last_limit = limit
        target = math.copysign(min(abs(requested_speed), limit), requested_speed)
        # Direction changes must first stop; never inject speed above TEB command.
        if self.speed*target < 0:
            self.speed = 0.0
        elif self.acceleration_ramp and abs(target) > abs(self.speed):
            self.speed = math.copysign(min(abs(target), abs(self.speed)+self.acceleration*dt), target)
        else:
            self.speed = target
        return self.speed


def path_preview(points):
    p = np.asarray(points, dtype=float)
    if p.ndim != 2 or p.shape[1] != 2 or len(p) < 3 or not np.isfinite(p).all(): return None
    lengths = np.linalg.norm(np.diff(p, axis=0), axis=1)
    keep = np.r_[True, lengths > 1e-5]
    p = p[keep]
    if len(p) < 3: return None
    lengths = np.linalg.norm(np.diff(p, axis=0), axis=1)
    distance = np.r_[0, np.cumsum(lengths)]
    if distance[-1] < 0.45: return None
    sample_s = np.arange(0, min(distance[-1], 5.0), 0.15)
    samples = np.column_stack([np.interp(sample_s, distance, p[:, i]) for i in (0, 1)])
    # A 0.6m physical span reduces sensitivity to tiny path discretization changes.
    out = []
    for i in range(2, len(samples)-2):
        a, b = samples[i]-samples[i-2], samples[i+2]-samples[i]
        denom = np.linalg.norm(a)*np.linalg.norm(b)*np.linalg.norm(a+b)
        if denom > 1e-8:
            curve = 2*abs(a[0]*b[1]-a[1]*b[0])/denom
            out.append((float(max(0, sample_s[i]-0.3)), float(curve)))
    return out if out else None
