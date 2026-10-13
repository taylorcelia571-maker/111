"""Bounded pause before accelerating again after a risk-induced speed drop."""
import math


class AccelerationRecovery:
    RISK_REASONS = frozenset(('laser', 'outward_error_recovery', 'steering_slew',
                              'steering_alignment', 'tracking_error'))

    def __init__(self, hold_seconds=0.0):
        if not math.isfinite(hold_seconds) or not 0 <= hold_seconds <= .4:
            raise ValueError('speed_recovery_hold must be between 0 and .4 seconds')
        self.hold_seconds = float(hold_seconds)
        self.reset()

    def reset(self):
        self.remaining = 0.0
        self.previous = 0.0

    def step(self, requested, reason, dt):
        if not math.isfinite(requested) or requested < 0 or not math.isfinite(dt) or not 0 < dt <= .3:
            raise ValueError('Invalid acceleration recovery input')
        self.remaining = max(0.0, self.remaining-dt)
        if reason in self.RISK_REASONS and requested < self.previous-.02:
            self.remaining = self.hold_seconds
        # Never smooth or postpone braking, nor exceed any upstream speed cap.
        output = min(requested, self.previous) if self.remaining > 0 else requested
        self.previous = output
        return output
