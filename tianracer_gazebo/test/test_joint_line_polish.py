import math
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import Mock, patch
from xml.etree import ElementTree as ET
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from race_line import ClosedPath
from joint_line_polish import polish_line


class JointLineTests(unittest.TestCase):
    def setUp(self):
        angle = np.linspace(0, 2 * math.pi, 200)
        self.seed = ClosedPath(np.column_stack((6 * np.cos(angle), 6 * np.sin(angle))), spacing=.1)
        self.anchors = self.seed.points[::100]
        self.planner = SimpleNamespace(margin=.34, score=Mock(return_value=20.),
            clearance_at=lambda points: np.full(len(points), 1.), feasible=Mock(return_value=None))
        self.budgets = dict(max_speed=10., lateral_accel=9.94, acceleration=7.3,
            braking=3.7, steering_rate=2., yaw_rate=4.4, response_delay=.25, speed_planning='delayed_braking')

    def fake_solver(self, success=True):
        def solve(function, initial, **kwargs):
            self.assertEqual(kwargs['method'], 'SLSQP')
            return SimpleNamespace(success=success, message='test solver', nit=1,
                                   x=np.full(len(initial), .01))
        return solve

    def test_invalid_strength_rejected(self):
        for value in (0, .09, 1.01, True, math.nan, math.inf):
            with self.subTest(value=value), self.assertRaises(ValueError):
                polish_line(self.planner, self.seed, self.anchors, self.budgets, strength=value)
        self.planner.score.assert_not_called()

    def test_solver_failure_keeps_original_reference(self):
        with patch('joint_line_polish.minimize', side_effect=self.fake_solver(False)):
            result, report = polish_line(self.planner, self.seed, self.anchors, self.budgets)
        self.assertIs(result, self.seed)
        self.assertFalse(report['accepted'])
        self.planner.feasible.assert_not_called()

    def test_solver_success_never_bypasses_geometry_guard(self):
        with patch('joint_line_polish.minimize', side_effect=self.fake_solver()):
            result, report = polish_line(self.planner, self.seed, self.anchors, self.budgets)
        self.assertIs(result, self.seed)
        self.assertFalse(report['accepted'])
        self.assertEqual(self.planner.feasible.call_count, 3)
        self.assertTrue(all(not item['feasible'] for item in report['candidates']))

    def test_feasible_but_slower_never_replaces_baseline(self):
        self.planner.feasible.return_value = self.seed
        self.planner.score.side_effect = [20., 21., 20., 20.001]
        with patch('joint_line_polish.minimize', side_effect=self.fake_solver()):
            result, report = polish_line(self.planner, self.seed, self.anchors, self.budgets)
        self.assertIs(result, self.seed)
        self.assertFalse(report['accepted'])

    def test_accepted_candidate_preserves_start_and_budget(self):
        original = self.seed.points.copy()
        budget_copy = dict(self.budgets)
        self.planner.feasible.return_value = self.seed
        self.planner.score.side_effect = [20., 21., 19., 18.]
        with patch('joint_line_polish.minimize', side_effect=self.fake_solver()):
            result, report = polish_line(self.planner, self.seed, self.anchors, self.budgets)
        self.assertTrue(report['accepted'])
        self.assertEqual(report['protected_input_displacement'], 0.)
        self.assertEqual(report['selected_model_lap_seconds'], 18.)
        self.assertFalse(report['official_result'])
        self.assertEqual(self.budgets, budget_copy)
        np.testing.assert_array_equal(self.seed.points, original)

    def test_all_features_default_off_and_legacy_forces_off(self):
        launch = Path(__file__).resolve().parents[1] / 'launch'
        for filename in ('planned_race.launch', 'final_speedup.launch', 'final_speedup_demo.launch'):
            root = ET.parse(launch / filename).getroot()
            args = {item.attrib['name']: item.attrib.get('default') for item in root.findall('arg')}
            self.assertEqual(args['route_joint_polish'], 'false')
            self.assertEqual(args['runtime_path_cache'], 'false')
            self.assertEqual(args['steering_preview_mode'], 'legacy')
        root = ET.parse(launch / 'final_speedup.launch').getroot()
        forwarded = {item.attrib['name']: item.attrib['value'] for item in root.find('include').findall('arg')}
        for name in ('route_joint_polish', 'runtime_path_cache', 'steering_preview_mode'):
            self.assertIn("arg('legacy_control')", forwarded[name])


if __name__ == '__main__':
    unittest.main()
