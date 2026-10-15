import copy
import math
from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from race_line import ClosedPath
from predictive_follower import PredictiveFollower
from track_memory import CyclicTrackMemory


def make_follower(**overrides):
    angle = np.linspace(0, 2 * math.pi, 700)
    path = ClosedPath(np.column_stack((6 * np.cos(angle), 6 * np.sin(angle))), spacing=.04)
    options = dict(max_speed=10., lateral_accel=9.94, acceleration=7.3, braking=3.7,
                   yaw_rate=4.4, steering_rate=2., response_delay=.25,
                   speed_planning='delayed_braking', lateral_control='race_feedback',
                   steering_feedback_gain=.25, steering_preview_time=.12,
                   startup_speed=2., startup_acceleration=2.5,
                   startup_release_early=True, fast_start=True, start_blend=True)
    options.update(overrides)
    return PredictiveFollower(path, **options)


class TrackMemoryTests(unittest.TestCase):
    def test_cyclic_queries_match_brute_force_including_seam(self):
        follower = make_follower(runtime_path_cache=True)
        path, cache = follower.path, follower.track_memory
        for i in (-2, 0, 3, path.n - 1, path.n + 1):
            for count in (0, 1, 2, 3, 7, 32, path.n - 1):
                ids = (i + np.arange(count + 1)) % path.n
                self.assertEqual(cache.minimum_speed(i, count), float(path.speeds[ids].min()))
                self.assertEqual(cache.maximum_curvature(i, count), float(abs(path.curvature[ids]).max()))

    def test_projection_is_bit_identical_across_five_laps(self):
        f = make_follower(runtime_path_cache=True)
        for lap in range(5):
            for s in np.linspace(-.03, f.path.length + .03, 60) + lap * f.path.length:
                x, y = f.path.sample(s) + [.02, -.01]
                begin, end = s - .15, s + .7
                self.assertEqual(f.path.project(x, y, begin, end), f.track_memory.project(x, y, begin, end))

    def test_changed_reference_fails_closed(self):
        f = make_follower(runtime_path_cache=True)
        with self.assertRaises(ValueError):
            f.path.points[0] += 1.
        f.path.speeds = f.path.speeds.copy()
        with self.assertRaisesRegex(ValueError, 'rebuild while stationary'):
            f.track_memory.minimum_speed(0, 0)

    def test_prepare_deepcopy_preserves_cache_reference_and_isolation(self):
        original = make_follower(runtime_path_cache=True)
        probe = copy.deepcopy(original)
        probe.track_memory.validate()
        self.assertIs(probe.track_memory.path, probe.path)
        self.assertIsNot(probe.path.points, original.path.points)
        self.assertFalse(probe.path.points.flags.writeable)
        original.track_memory.validate()

    def test_cached_and_original_follower_outputs_match_over_five_laps(self):
        ordinary = make_follower()
        cached = make_follower(runtime_path_cache=True)
        for f in (ordinary, cached):
            f.start(6., 0., math.pi / 2)
        for s in np.linspace(0., 5 * ordinary.path.length, 2500):
            x, y = ordinary.path.sample(s)
            tangent = ordinary.path.sample(s + .08) - ordinary.path.sample(s - .08)
            yaw = math.atan2(tangent[1], tangent[0])
            outputs = []
            for f in (ordinary, cached):
                f.measured_steer = math.atan(.26 / 6)
                f.measured_steer_age = 0.
                outputs.append(f.step(x, y, yaw, 4., .02))
            self.assertEqual(outputs[0], outputs[1])
            self.assertEqual(ordinary.progress, cached.progress)
            self.assertEqual(ordinary.error, cached.error)

    def test_invalid_memory_query_and_configuration(self):
        f = make_follower(runtime_path_cache=True)
        for count in (-1, f.path.n, 1.5):
            with self.assertRaises(ValueError):
                f.track_memory.minimum_speed(0, count)
        for overrides in ({'runtime_path_cache': 'true'}, {'steering_preview_mode': 'blind_replay'},
                          {'steering_preview_mode': 'curvature_adaptive', 'lateral_control': 'pursuit'}):
            with self.assertRaises(ValueError):
                make_follower(**overrides)

    def test_hard_tracking_stop_still_triggers_with_cache(self):
        f = make_follower(runtime_path_cache=True)
        f.start(6., 0., math.pi / 2)
        with self.assertRaisesRegex(ValueError, 'Tracking error exceeds safe corridor'):
            f.step(6.2, 0., math.pi / 2, 1., .02)
        self.assertEqual(f.speed, 0.)

    def test_cached_plan_report_keeps_reference_and_can_evaluate_sensitivity(self):
        f = make_follower(runtime_path_cache=True)
        before = f.path.fingerprint()
        report = f.plan_report()
        self.assertEqual(report['track_memory']['geometry_sha256'], before)
        self.assertIn('same_route_parameter_sensitivity', report)
        f.track_memory.validate()


class SteeringPreviewTests(unittest.TestCase):
    def test_legacy_cap_and_no_first_lap_warmup(self):
        f = make_follower(runtime_path_cache=True)
        f.start(6., 0., math.pi / 2)
        self.assertAlmostEqual(f.steering_advance(0., 6., .02), .30)
        self.assertEqual(f.track_memory.report()['warmup_laps'], 0)
        self.assertFalse(f.track_memory.report()['replay_commands'])

    def test_adaptive_preview_keeps_departure_connector_and_startup_unchanged(self):
        f = make_follower(steering_preview_mode='curvature_adaptive')
        f.progress = f.origin = 0.
        for departure, limited, offset in ((True, False, None), (False, True, None),
                                           (False, False, np.array([.07, 0.]))):
            f.departure, f.launch_limited, f.connector_offset = departure, limited, offset
            self.assertAlmostEqual(f.steering_advance(0., 5., .02), .3)

    def test_high_speed_preview_scales_and_changes_continuously(self):
        f = make_follower(steering_preview_mode='curvature_adaptive')
        f.progress = f.origin = 0.
        f.departure = f.launch_limited = False
        f.connector_offset = None
        previous = .3
        for _ in range(30):
            advance = f.steering_advance(0., 5., .02)
            self.assertLessEqual(abs(advance - previous), .04 + 1e-12)
            previous = advance
        self.assertAlmostEqual(advance, .6)
        self.assertEqual(f.response_delay, .25)
        self.assertEqual(f.max_error, .15)

    def test_restart_resets_preview_state_not_lap_seam(self):
        f = make_follower(steering_preview_mode='curvature_adaptive')
        f.last_preview_advance = .6
        f.start(6., 0., math.pi / 2)
        self.assertIsNone(f.last_preview_advance)
        f.progress = f.origin + f.path.length
        f.departure = f.launch_limited = False
        f.connector_offset = None
        f.steering_advance(f.progress, 5., .02)
        self.assertIsNotNone(f.last_preview_advance)


if __name__ == '__main__':
    unittest.main()
