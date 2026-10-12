"""Response-aware route selection and delayed tracking on the repository map."""
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from race60_audit import load_course, rear_path, simulate
from predictive_follower import PredictiveFollower


class ResponseRouteTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.planner, cls.anchors = load_course(Path(__file__).resolve().parents[1])
        cls.budgets = dict(max_speed=2.2, lateral_accel=3.2, acceleration=1.5,
                           braking=2., yaw_rate=2.5, steering_rate=2.)
        cls.base = cls.planner.build(cls.anchors, optimize_time=True,
                                    response_delay=.25, **cls.budgets)
        cls.refined = cls.planner.build(cls.anchors, optimize_time=True,
                                       refine_line=True, response_delay=.25, **cls.budgets)
        cls.selection = dict(cls.planner.selection)

    def test_refinement_reduces_same_budget_response_proxy(self):
        seconds=[]
        for center in (self.base,self.refined):
            f=PredictiveFollower(rear_path(center),response_delay=.25,**self.budgets)
            seconds.append(float(np.sum(f.path.ds/f.preview_speed_profile())))
        self.assertLess(seconds[1], seconds[0]-.1)
        self.assertAlmostEqual(seconds[1], self.selection['estimated_lap_seconds'])
        self.assertEqual(self.selection['scoring_response_delay'],.25)

    def test_refined_route_retains_clearance_and_waypoint_order(self):
        self.assertGreaterEqual(self.planner.segment_check(self.refined.points),.26)
        positions=[]
        for anchor in self.anchors:
            s,error=self.refined.project(*anchor)
            self.assertLessEqual(error,.4)
            positions.append(s)
        increments=np.diff(np.r_[positions,positions[0]+self.refined.length])%self.refined.length
        self.assertAlmostEqual(float(increments.sum()), self.refined.length, places=6)

    def test_refined_five_model_loops_for_three_actuator_lags(self):
        for lag in (.08,.15,.25):
            with self.subTest(lag=lag):
                row=simulate(self.refined,self.planner,self.budgets,.25,lag)
                self.assertTrue(row['completed'],row)
                self.assertLess(row['peak_error'],.15)
                self.assertGreaterEqual(row['minimum_center_clearance'],.23)
                self.assertFalse(row['official_result'])

    def test_same_budget_delayed_model_is_faster_than_unrefined(self):
        baseline=simulate(self.base,self.planner,self.budgets,.25,.15)
        candidate=simulate(self.refined,self.planner,self.budgets,.25,.15)
        self.assertTrue(baseline['completed'])
        self.assertTrue(candidate['completed'])
        self.assertLess(candidate['seconds'], baseline['seconds'])

    def test_response_proxy_is_cyclic_and_does_not_mutate_envelope(self):
        f=PredictiveFollower(rear_path(self.base),response_delay=.25,**self.budgets)
        f.path.speeds[:]=2.;f.path.speeds[0]=.5
        original=f.path.speeds.copy()
        preview=f.preview_speed_profile()
        self.assertEqual(preview[-1],.5)
        self.assertTrue(np.array_equal(f.path.speeds,original))
        f.response_delay=0
        self.assertTrue(np.array_equal(f.preview_speed_profile(),original))

    def test_invalid_response_budget_rejected_before_search(self):
        for value in (-.1,.51,float('nan')):
            with self.assertRaises(ValueError):
                self.planner.build(self.anchors,response_delay=value)


if __name__=='__main__':unittest.main()
