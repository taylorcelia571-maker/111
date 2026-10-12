"""Models, partial runs and mismatched telemetry cannot pass the time target."""
import copy
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from check_race_result import analyze


class ResultCheckTests(unittest.TestCase):
    def setUp(self):
        limits = dict(max_speed=8, lateral_accel=6.5, acceleration=5, braking=3,
                      yaw_rate=3.8, steering_rate=2, response_delay=.25)
        self.plan = dict(reference_sha256='same', lateral_control='smooth_pursuit',
                         limits=limits, response_delay_seconds=.25,
                         model_five_seconds=55, speed_ceiling_binding=False)
        self.run = dict(reference_sha256='same', controller='smooth_pursuit',
                        limits=dict(limits), last_fault=None, observed_simulation_seconds=82,
                        reference_loop_seconds=[21, 16, 16, 16, 16],
                        time_by_state={'response_preview': dict(seconds=40, mean_measured_speed=2.4),
                                       'road:curve': dict(seconds=50)})

    def test_faster_model_does_not_pass_slower_actual_result(self):
        row = analyze(self.plan, self.run, 82, 5)
        self.assertFalse(row['supplied_result_meets_time_target'])
        self.assertEqual(row['seconds_above_target'], 22)
        self.assertEqual(row['reference_startup_excess_seconds'], 5)
        self.assertEqual(len(row['top_speed_restrictions']), 1)

    def test_fast_unfinished_run_does_not_pass(self):
        self.assertFalse(analyze(self.plan, self.run, 30, 2)['supplied_result_meets_time_target'])

    def test_partial_snapshot_does_not_validate_complete_official_run(self):
        self.run['reference_loop_seconds'] = [16]
        self.run['observed_simulation_seconds'] = 20
        self.assertFalse(analyze(self.plan, self.run, 59, 5)['supplied_result_meets_time_target'])

    def test_mismatched_geometry_and_parameters_rejected(self):
        for key, value in [('reference_sha256', 'different'), ('limits', dict(max_speed=9))]:
            run = copy.deepcopy(self.run); run[key] = value
            self.assertFalse(analyze(self.plan, run, 59, 5)['supplied_result_meets_time_target'])

    def test_protective_stop_cannot_pass(self):
        self.run['last_fault'] = dict(reason='wall clearance')
        self.assertFalse(analyze(self.plan, self.run, 59, 5)['supplied_result_meets_time_target'])

    def test_different_response_delay_cannot_pass(self):
        self.run['limits']['response_delay'] = .15
        self.assertFalse(analyze(self.plan, self.run, 59, 5)['supplied_result_meets_time_target'])

    def test_strict_time_target_and_evidence_label(self):
        self.assertFalse(analyze(self.plan, self.run, 60, 5)['supplied_result_meets_time_target'])
        result = analyze(self.plan, self.run, 59.999, 5)
        self.assertTrue(result['supplied_result_meets_time_target'])
        self.assertFalse(result['independently_verified_official_result'])

    def test_invalid_time_rejected(self):
        for seconds in (float('nan'), float('inf'), -1, 0, True):
            with self.assertRaises(ValueError): analyze(self.plan, self.run, seconds, 5)


if __name__ == '__main__': unittest.main()
