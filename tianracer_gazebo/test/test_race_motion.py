"""Offline command-dynamics checks; no ROS or simulated lap claims."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from race_motion import RaceMotion


class RaceMotionTests(unittest.TestCase):
    def test_straight_acceleration_and_speed_limit(self):
        motion = RaceMotion()
        previous = 0.0
        for _ in range(100):
            speed, steer = motion.step(9.0, 0.0, 0.02)
            self.assertLessEqual(speed - previous, motion.acceleration * 0.02 + 1e-10)
            self.assertLessEqual(speed, 2.4)
            self.assertEqual(steer, 0.0)
            previous = speed
        self.assertAlmostEqual(speed, 2.4)

    def test_turn_entry_and_exit_obey_curvature_limit(self):
        motion = RaceMotion()
        for _ in range(100):
            motion.step(2.4, 0.0, 0.02)
        for target in [0.45] * 50 + [0.0] * 50 + [-0.45] * 50:
            speed, steer = motion.step(2.4, target, 0.02)
            curvature = max(abs(math.tan(target)), abs(math.tan(steer))) / motion.wheelbase
            self.assertLessEqual(speed ** 2 * curvature, motion.lateral_accel + 1e-9)

    def test_steering_rate_limit_and_continuity(self):
        motion = RaceMotion()
        previous = 0.0
        for target in [0.45] * 20 + [-0.45] * 20:
            _, steer = motion.step(2.4, target, 0.02)
            self.assertLessEqual(abs(steer - previous), motion.steer_rate * 0.02 + 1e-10)
            self.assertLessEqual(abs(steer), motion.max_steer)
            previous = steer

    def test_zero_and_invalid_commands_stop_immediately(self):
        for command in [(0.0, 0.2, 0.02), (float('nan'), 0, 0.02),
                        (1, float('inf'), 0.02), (1, 0, -0.1), (1, 0, 0)]:
            motion = RaceMotion()
            motion.speed = 2.4
            self.assertEqual(motion.step(*command)[0], 0.0)

    def test_reverse_changes_sign_only_after_stopping(self):
        motion = RaceMotion()
        motion.speed = 1.0
        for _ in range(12):
            speed, _ = motion.step(-1.0, 0.0, 0.02)
            self.assertGreaterEqual(speed, 0.0)
        for _ in range(50):
            motion.step(-1.0, 0.0, 0.02)
        self.assertAlmostEqual(motion.speed, -1.0)

    def test_invalid_limits_rejected(self):
        for kwargs in ({'wheelbase': 0}, {'max_speed': -1}, {'steer_tau': float('nan')},
                       {'max_steer': math.pi / 2}, {'lateral_accel': float('inf')}):
            with self.assertRaises(ValueError):
                RaceMotion(**kwargs)


if __name__ == '__main__':
    unittest.main()
