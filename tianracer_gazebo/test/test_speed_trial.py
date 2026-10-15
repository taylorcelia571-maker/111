import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / 'scripts'))
from speed_trial import choose_session, inherited_parameters, launch_arguments, make_recipe


class TrialTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.session = self.root / 'final71' / 'source70'
        self.session.mkdir(parents=True)
        self.parameters = dict(route_margin=.34, response_delay=.25, max_error=.15,
            max_speed=10., steering_rate=2., lateral_control='race_feedback', speed_planning='delayed_braking',
            lateral_accel=9.94, acceleration=7.3, braking=3.7, yaw_rate=4.4,
            startup_speed=2., startup_acceleration=2.5, steering_feedback_gain=.25,
            steering_preview_time=.12, heading_feedback_gain=.8, cross_track_gain=.8,
            line_search_level=3, predict_pose=False, startup_connector_length=2.5,
            speed_recovery_hold=0., route_control_scoring=False,
            route_joint_polish=True, route_polish_strength=.6)
        self.run = dict(last_fault=None, run_sequence=1, configuration={})
        self.save()
        (self.session / 'commit.txt').write_text('source-commit')
        (self.root / 'final_last_session.txt').write_text(str(self.session))

    def save(self):
        (self.session / 'run_report.json').write_text(json.dumps(self.run))
        (self.session / 'params.yaml').write_text(yaml.safe_dump(self.parameters))

    def test_baseline_inherits_actual_strength_instead_of_assuming_half(self):
        recipe = make_recipe(self.session, 'baseline')
        self.assertEqual(recipe['parameters']['route_polish_strength'], .6)
        self.assertEqual(recipe['changed'], {})
        self.assertFalse(recipe['official_under_60_verified'])

    def test_algorithm_changes_only_one_setting(self):
        recipe = make_recipe(self.session, 'algorithm')
        self.assertEqual(list(recipe['changed']), ['route_speed_refine'])
        self.assertEqual(recipe['parameters']['lateral_accel'], 9.94)
        self.assertEqual(recipe['parameters']['braking'], 3.7)

    def test_lateral_steps_are_small_and_explicit_opt_in(self):
        first = make_recipe(self.session, 'lateral_101')
        self.assertTrue(first['budget_trial'])
        self.assertEqual(list(first['changed']), ['lateral_accel'])
        self.assertIn('budget_trial:=true', launch_arguments(first))
        with self.assertRaisesRegex(ValueError, 'at most 0.2'):
            make_recipe(self.session, 'lateral_102')
        self.parameters['lateral_accel'] = 10.1
        self.save()
        self.assertEqual(make_recipe(self.session, 'lateral_102')['parameters']['lateral_accel'], 10.2)

    def test_acceleration_is_one_small_change(self):
        recipe = make_recipe(self.session, 'accel_75')
        self.assertEqual(list(recipe['changed']), ['acceleration'])
        self.assertEqual(recipe['parameters']['acceleration'], 7.5)

    def test_fault_and_multiple_starts_rejected(self):
        for run in ({'last_fault': {'reason': 'stop'}}, {}, {'last_fault': None, 'run_sequence': 2}):
            self.run = run
            self.save()
            with self.assertRaises(ValueError):
                inherited_parameters(self.session)

    def test_boundary_or_report_mismatch_rejected(self):
        self.parameters['response_delay'] = .2
        self.save()
        with self.assertRaisesRegex(ValueError, 'boundary'):
            inherited_parameters(self.session)
        self.parameters['response_delay'] = .25
        self.run['configuration']['acceleration'] = 8.
        self.save()
        with self.assertRaisesRegex(ValueError, 'mismatch'):
            inherited_parameters(self.session)

    def test_invalid_observation_not_promoted_to_baseline(self):
        (self.session / 'referee_observation.json').write_text(json.dumps(dict(valid_5_of_5=True, any_incident=True)))
        with self.assertRaisesRegex(ValueError, 'incident'):
            inherited_parameters(self.session)

    def test_source_route_checksum_mismatch_rejected(self):
        self.run['reference_sha256'] = 'run-a'
        self.save()
        (self.session / 'planned_course_report.json').write_text(json.dumps({'reference_sha256': 'plan-b'}))
        with self.assertRaisesRegex(ValueError, 'checksum'):
            inherited_parameters(self.session)

    def test_best_pointer_and_active_recording(self):
        (self.root / 'best_race_session.txt').write_text(str(self.session))
        self.assertEqual(choose_session(self.root, 'best'), self.session)
        (self.session / 'control.bag.active').write_bytes(b'x')
        with self.assertRaisesRegex(ValueError, 'active'):
            choose_session(self.root, 'best')

    def test_no_manual_best_is_inferred_from_reference_time(self):
        with self.assertRaisesRegex(ValueError, 'No saved best'):
            choose_session(self.root, 'best')

    def test_default_cli_is_dry_and_does_not_launch_ros(self):
        script = PACKAGE / 'scripts' / 'speed_trial.py'
        result = subprocess.run([sys.executable, str(script), '--base', 'latest', '--profile', 'algorithm',
                                 '--log-root', str(self.root)], text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('Dry-run only', result.stdout)
        self.assertFalse((self.root / 'speed_trial_recipes').exists())


class ShellBudgetTests(unittest.TestCase):
    def parse_only(self, *args):
        source = (PACKAGE / 'scripts' / 'run_final_speedup.sh').read_text()
        prefix = source.split('\nset +u\n', 1)[0]
        # Execute actual argument validation only; never source ROS or launch.
        code = prefix + '\nprintf "%s\\n" "${launch_overrides[@]}"\n'
        return subprocess.run(['bash', '-c', code, 'budget-parser', 'baseline', *args],
                              text=True, capture_output=True)

    def test_default_gate_preserved(self):
        self.assertEqual(self.parse_only('lateral_accel:=10.1').returncode, 2)

    def test_opt_in_is_order_independent_and_not_forwarded_to_ros(self):
        for args in (('budget_trial:=true', 'lateral_accel:=10.1'),
                     ('lateral_accel:=10.1', 'budget_trial:=true')):
            result = self.parse_only(*args)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertNotIn('budget_trial:=', result.stdout)

    def test_upper_bound_and_braking_gate_cannot_be_bypassed(self):
        for args in (('budget_trial:=true', 'lateral_accel:=10.21'),
                     ('budget_trial:=true', 'braking:=3.8'), ('response_delay:=0.1',),
                     ('route_margin:=0.2',), ('budget_trial:=true', 'budget_trial:=false')):
            self.assertEqual(self.parse_only(*args).returncode, 2, args)


if __name__ == '__main__':
    unittest.main()
