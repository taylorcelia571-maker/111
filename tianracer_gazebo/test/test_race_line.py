"""Closed-path dynamics regression tests; not Gazebo or official race results."""
import math
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from race_line import ClosedPath, LineFollower, clearance_speed


def circle(radius=3):
    t = np.linspace(0, 2 * math.pi, 500)
    return np.column_stack([radius * np.cos(t), radius * np.sin(t)])


class RaceLineTests(unittest.TestCase):
    def test_reject_open_short_and_jump_routes(self):
        for p in (circle()[:250], circle(0.5), [[0, 0]] * 20,
                  np.vstack([circle(), [99, 99]])):
            with self.assertRaises(ValueError): ClosedPath(p)

    def test_cyclic_speed_obeys_corner_accel_and_braking(self):
        t = np.linspace(0, 2 * math.pi, 800)
        path = ClosedPath(np.column_stack([6 * np.cos(t), 2 * np.sin(t)]))
        speeds = path.plan_speed(4, 3, 2.5, 3.5)
        self.assertTrue(np.all(speeds <= 4))
        self.assertTrue(np.all(speeds ** 2 * np.abs(path.curvature) <= 3 + 1e-8))
        self.assertTrue(np.all(speeds ** 2 <= np.roll(speeds, -1) ** 2 + 7 * path.ds + 1e-8))
        self.assertTrue(np.all(speeds ** 2 <= np.roll(speeds, 1) ** 2 + 5 * path.ds + 1e-8))
        self.assertGreater(speeds.max() - speeds.min(), 0.5)

    def test_braking_starts_before_a_tight_curve(self):
        t = np.linspace(0, 2 * math.pi, 800)
        path = ClosedPath(np.column_stack([6 * np.cos(t), 2 * np.sin(t)]))
        speed = path.plan_speed(4, 3, 2.5, 3.5)
        low = int(np.argmin(speed))
        before = (low - int(0.8 / path.ds)) % path.n
        self.assertLess(speed[before], 4)

    def test_two_continuous_kinematic_laps(self):
        path = ClosedPath(circle())
        controller = LineFollower(path)
        x, y, yaw, speed = 3.0, 0.0, math.pi / 2, 0.0
        dt = 0.02
        max_error = 0
        for i in range(2000):
            speed, steer = controller.step(x, y, yaw, speed, dt)
            yaw += speed * math.tan(steer) / controller.wheelbase * dt
            x += speed * math.cos(yaw) * dt
            y += speed * math.sin(yaw) * dt
            max_error = max(max_error, controller.error)
            if controller.progress - controller.origin >= 2 * path.length: break
        else: self.fail('Did not finish two reference laps')
        self.assertLess(max_error, 0.15)
        self.assertLess((i + 1) * dt, 2 * path.length / 1.8)

    def test_forward_projection_does_not_jump_to_distant_arc(self):
        path = ClosedPath(circle())
        s, _ = path.project(-3, 0, 0, 0.3)
        self.assertLessEqual(s, 0.3)

    def test_departure_and_wrong_heading_stop(self):
        controller = LineFollower(ClosedPath(circle()))
        with self.assertRaises(ValueError): controller.start(3, 0, -math.pi / 2)
        controller.start(3, 0, math.pi / 2)
        with self.assertRaises(ValueError): controller.step(10, 0, math.pi / 2, 1, 0.02)
        self.assertEqual(controller.speed, 0)

    def test_invalid_dt_rejected(self):
        controller = LineFollower(ClosedPath(circle()))
        for dt in (0, 0.5, float('nan')):
            with self.assertRaises(ValueError): controller.step(3, 0, math.pi / 2, 1, dt)

    def test_laser_blockage_invalid_and_clear(self):
        angles = np.linspace(-math.pi / 2, math.pi / 2, 181)
        def limit(r): return clearance_speed(r, angles[0], angles[1]-angles[0], 0.05, 10, 3, 0)
        self.assertEqual(limit(np.zeros(181)), 0)
        self.assertTrue(math.isinf(limit(np.full(181, np.inf))))
        r = np.full(181, np.inf)
        r[90] = 0.2
        self.assertEqual(limit(r), 0)
        r[90] = 1.0
        self.assertLess(limit(r), 3)

    def test_laser_offset_is_accounted_for(self):
        r = np.full(181, np.inf)
        r[90] = 0.3
        result = clearance_speed(r, -math.pi / 2, math.pi / 180, 0.05, 10, 2, 0,
                                 laser_x=-0.2)
        self.assertEqual(result, 0)


if __name__ == '__main__': unittest.main()
