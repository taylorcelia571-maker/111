"""Ackermann wheel rolling feedforward using the unchanged model geometry."""
import math


def wheel_commands(speed, steer, feedforward=False):
    if not math.isfinite(speed) or not math.isfinite(steer):
        return {name: 0.0 for name in ('lr', 'rr', 'lf', 'rf', 'left', 'right')}
    if not feedforward:
        # Preserve historical direct mode, including its original axle length.
        length = 0.265
        k = math.tan(steer) / length
        return dict(lr=speed/0.032, rr=speed/0.032, lf=speed/0.032, rf=speed/0.032,
                    left=math.atan2(length*k, 1-0.065*k),
                    right=math.atan2(length*k, 1+0.065*k))
    # URDF: hinges x=+0.13, rear axle x=-0.13, rear centers y=+-0.09,
    # hinge y=+-0.065, front wheel offset +-0.025, wheel radius 0.032.
    length, radius, hinge_y, rear_y, offset = 0.26, 0.032, 0.065, 0.09, 0.025
    steer = max(-0.5, min(0.5, steer))  # Inner hinge remains within model +-0.6.
    k = math.tan(steer) / length
    yaw_rate = speed * k
    left = math.atan2(length*k, 1-hinge_y*k)
    right = math.atan2(length*k, 1+hinge_y*k)
    # Offset wheels also move as their steering hinges rotate around the ICR.
    return dict(lr=speed*(1-rear_y*k)/radius, rr=speed*(1+rear_y*k)/radius,
                lf=(speed*math.hypot(1-hinge_y*k, length*k)-yaw_rate*offset)/radius,
                rf=(speed*math.hypot(1+hinge_y*k, length*k)+yaw_rate*offset)/radius,
                left=left, right=right)


def equivalent_steer(left, right, length=0.26):
    a, b = math.tan(left), math.tan(right)
    return 0.5*(math.atan(length*a/(length+0.065*a))
                + math.atan(length*b/(length-0.065*b)))
