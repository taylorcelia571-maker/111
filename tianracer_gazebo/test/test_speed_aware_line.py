import math
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import Mock, patch
from xml.etree import ElementTree as ET
import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / 'scripts'))
from race_line import ClosedPath
from speed_aware_line import refine_speed_line, sector_speed_report


class SpeedLineTests(unittest.TestCase):
    def setUp(self):
        angle = np.linspace(0, 2 * math.pi, 200)
        self.seed = ClosedPath(np.column_stack((6 * np.cos(angle), 6 * np.sin(angle))), spacing=.1)
        self.anchors = self.seed.points[::100]
        self.alternative = ClosedPath(np.vstack([self.seed.points, self.seed.points[0]]), spacing=.1)
        self.planner = SimpleNamespace(margin=.34, score=Mock(return_value=20.),
            clearance_at=lambda points: np.full(len(points), 1.),
            segment_check=lambda points: 1., feasible=Mock(return_value=None))
        self.budgets = dict(max_speed=10., lateral_accel=9.94, acceleration=7.3,
            braking=3.7, steering_rate=2., yaw_rate=4.4, response_delay=.25, speed_planning='delayed_braking')

    def solver(self, objective, initial, **kwargs):
        self.assertEqual(kwargs['options']['eps'], .002)
        objective(initial)
        objective(np.full(len(initial), .01))
        return SimpleNamespace(success=False, message='Iteration limit reached', nit=1)

    def test_invalid_iteration_budgets_rejected(self):
        for value in (0, 11, 1.5, True):
            with self.assertRaises(ValueError):
                refine_speed_line(self.planner, self.seed, self.anchors, self.budgets, value)

    def test_no_gain_keeps_original_identity(self):
        with patch('speed_aware_line.minimize', side_effect=self.solver):
            result, report = refine_speed_line(self.planner, self.seed, self.anchors, self.budgets)
        self.assertIs(result, self.seed)
        self.assertFalse(report['accepted'])
        self.planner.feasible.assert_not_called()

    def test_faster_surrogate_never_bypasses_geometry(self):
        self.planner.score.side_effect = lambda route, *args: 20. if route is self.seed else 10.
        with patch('speed_aware_line.minimize', side_effect=self.solver):
            result, report = refine_speed_line(self.planner, self.seed, self.anchors, self.budgets)
        self.assertIs(result, self.seed)
        self.assertFalse(report['accepted'])
        self.assertGreater(self.planner.feasible.call_count, 0)

    def test_validated_iterate_can_survive_iteration_limit_without_changing_budget(self):
        self.planner.score.side_effect = lambda route, *args: 20. if route is self.seed else 10.
        self.planner.feasible.return_value = self.alternative
        original = self.seed.points.copy()
        budgets = dict(self.budgets)
        with patch('speed_aware_line.minimize', side_effect=self.solver):
            result, report = refine_speed_line(self.planner, self.seed, self.anchors, self.budgets)
        self.assertIs(result, self.alternative)
        self.assertTrue(report['accepted'])
        self.assertFalse(report['solver_success'])
        self.assertFalse(report['official_result'])
        self.assertEqual(report['protected_input_displacement'], 0.)
        self.assertEqual(self.budgets, budgets)
        np.testing.assert_array_equal(self.seed.points, original)

    def test_optimizer_exception_falls_back_to_valid_original(self):
        with patch('speed_aware_line.minimize', side_effect=ValueError('synthetic solver failure')):
            result, report = refine_speed_line(self.planner, self.seed, self.anchors, self.budgets)
        self.assertIs(result, self.seed)
        self.assertFalse(report['accepted'])

    def test_faster_but_sharper_reference_is_not_accepted(self):
        self.planner.score.side_effect = lambda route, *args: 20. if route is self.seed else 10.
        self.planner.feasible.return_value = self.alternative
        metrics = lambda route: dict(curvature=.2, steering_gradient=.1) if route is self.seed else dict(curvature=2., steering_gradient=1.)
        with patch('speed_aware_line.reference_peaks', side_effect=metrics), patch('speed_aware_line.minimize', side_effect=self.solver):
            result, report = refine_speed_line(self.planner, self.seed, self.anchors, self.budgets)
        self.assertIs(result, self.seed)
        self.assertFalse(report['accepted'])

    def test_restart_portfolio_cannot_bypass_original_feasibility(self):
        self.planner.score.side_effect = lambda route, *args: 20. if route is self.seed else 10.
        with patch('joint_line_polish.polish_line', return_value=(self.alternative, {'accepted': True})), patch('speed_aware_line.minimize', side_effect=self.solver):
            result, report = refine_speed_line(self.planner, self.seed, self.anchors, self.budgets,
                                               source_seed=self.seed)
        self.assertIs(result, self.seed)
        self.assertTrue(all(not item['accepted'] for item in report['portfolio']))

    def test_sector_time_and_length_are_disjoint_model_diagnostics(self):
        path = SimpleNamespace(curvature=np.array([0., .149, .15, .59, .6, 1.]),
                               speeds=np.array([4., 4., 3., 3., 2., 2.]), ds=.1)
        report = sector_speed_report(path)
        self.assertTrue(report['model_only'])
        self.assertAlmostEqual(sum(row['length_m'] for row in report['sectors'].values()), .6)
        self.assertAlmostEqual(sum(row['model_lap_seconds'] for row in report['sectors'].values()),
                               float(sum(path.ds / path.speeds)))

    def test_launch_defaults_and_legacy_keep_new_optimizer_off(self):
        for filename in ('planned_race.launch', 'final_speedup.launch', 'final_speedup_demo.launch'):
            root = ET.parse(PACKAGE / 'launch' / filename).getroot()
            args = {item.attrib['name']: item.attrib.get('default') for item in root.findall('arg')}
            self.assertEqual(args['route_speed_refine'], 'false')
        root = ET.parse(PACKAGE / 'launch' / 'final_speedup.launch').getroot()
        forwarded = {item.attrib['name']: item.attrib['value'] for item in root.find('include').findall('arg')}
        self.assertIn("arg('legacy_control')", forwarded['route_speed_refine'])


if __name__ == '__main__':
    unittest.main()
