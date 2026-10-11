"""ROS-independent shaping; angular input is TEB steering angle, not yaw rate."""
import math


class RaceMotion:
    def __init__(self, wheelbase=0.265, max_speed=2.4, max_steer=0.48,
                 lateral_accel=2.5, acceleration=2.5, deceleration=4.0,
                 steer_rate=1.8, steer_tau=0.06):
        values = (wheelbase, max_speed, max_steer, lateral_accel,
                  acceleration, deceleration, steer_rate, steer_tau)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError('Motion limits must be finite and positive')
        if max_steer >= math.pi / 2:
            raise ValueError('max_steer must be less than pi/2')
        self.wheelbase = wheelbase
        self.max_speed = max_speed
        self.max_steer = max_steer
        self.lateral_accel = lateral_accel
        self.acceleration = acceleration
        self.deceleration = deceleration
        self.steer_rate = steer_rate
        self.steer_tau = steer_tau
        self.speed = 0.0
        self.steer = 0.0

    def step(self, speed, steer, dt):
        if not all(math.isfinite(v) for v in (speed, steer, dt)) or dt <= 0:
            self.speed = 0.0
            return self.speed, self.steer
        dt = min(dt, 0.1)
        target_steer = max(-self.max_steer, min(self.max_steer, steer))
        change = (target_steer - self.steer) * (-math.expm1(-dt / self.steer_tau))
        change = max(-self.steer_rate * dt, min(self.steer_rate * dt, change))
        self.steer += change
        # Bound entry using requested curvature, exit using actual command.
        curvature = max(abs(math.tan(target_steer)),
                        abs(math.tan(self.steer))) / self.wheelbase
        limit = min(self.max_speed, math.sqrt(self.lateral_accel / max(curvature, 1e-9)))
        target_speed = max(-limit, min(limit, speed))
        if speed == 0:
            self.speed = 0.0  # Never delay a stop command.
        elif self.speed * target_speed < 0:
            self.speed = math.copysign(max(0.0, abs(self.speed) - self.deceleration * dt),
                                       self.speed)
        else:
            rate = self.acceleration if abs(target_speed) > abs(self.speed) else self.deceleration
            delta = max(-rate * dt, min(rate * dt, target_speed - self.speed))
            self.speed += delta
        # A newly tighter curvature bound takes priority over comfort braking.
        self.speed = max(-limit, min(limit, self.speed))
        return self.speed, self.steer
