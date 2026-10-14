"""Optional stationary route score; an ideal-control proxy, never race evidence."""
import math
import numpy as np
from braking_envelope import delayed_envelope


def controller_speed_profile(follower, iterations=3):
    """Include ideal race_feedback feedforward/compensation and command slew.

    Assumes perfect tracking, zero joint transport age and measured steering equal
    to the current reference angle. No actuator, tire, localization or laser model.
    Uses the unchanged response/braking budget once for each candidate cap set.
    This score is used only while stationary; it never changes live protection.
    """
    if follower.speed_planning != 'delayed_braking':
        raise ValueError('Controller route scoring requires delayed_braking')
    path = follower.path
    raw_caps = np.minimum.reduce(list(follower.speed_caps.values()))
    speed = path.speeds.copy()
    current_angle = np.arctan(follower.wheelbase * path.curvature)
    for _ in range(iterations):
        advance = np.minimum(2.5 * follower.steering_preview_time,
                             speed * follower.steering_preview_time)
        indexes = (np.arange(path.n) * path.ds + advance) / path.ds
        whole = np.floor(indexes).astype(int)
        fraction = indexes - whole
        curve = ((1 - fraction) * path.curvature[whole % path.n] +
                 fraction * path.curvature[(whole + 1) % path.n])
        desired = np.clip(np.arctan(follower.wheelbase * curve),
                          -follower.max_steer, follower.max_steer)
        target = np.clip(desired + follower.steering_feedback_gain *
                         (desired - current_angle),
                         -follower.max_steer, follower.max_steer)
        curvature = np.maximum.reduce((np.abs(curve), np.abs(path.curvature),
                                       np.abs(np.tan(target) / follower.wheelbase)))
        derivative = np.maximum(np.abs(np.roll(target, -1) - target),
                                np.abs(target - np.roll(target, 1))) / path.ds
        caps = np.minimum.reduce((raw_caps,
            np.sqrt(follower.lateral_accel / np.maximum(curvature, 1e-6)),
            follower.yaw_rate / np.maximum(curvature, 1e-6),
            follower.steering_rate / np.maximum(derivative, 1e-6)))
        trial = delayed_envelope(caps, path.ds, follower.braking,
                                 follower.acceleration, follower.response_delay)
        speed = np.minimum(speed, trial)
    if not np.isfinite(speed).all() or np.any(speed <= 0):
        raise ValueError('Invalid controller scoring profile')
    return speed
