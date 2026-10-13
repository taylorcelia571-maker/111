from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from race60_audit import load_course
from time_attack_audit import BASE


class AdvancedLineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.planner,cls.anchors=load_course(Path(__file__).resolve().parents[1])
        cls.planner.margin=.34;cls.planner.allowed=cls.planner.clearance>=.34
        cls.seed=cls.planner.build(cls.anchors,optimize_time=True,refine_line=True,
            response_delay=.25,speed_planning='legacy',**BASE)
        cls.budgets=(10,6.5,6.6,3.6,2,3.8,.25,'legacy')
        cls.result,cls.report=cls.planner.refine_multiscale(cls.seed,cls.anchors,.48,.26,
                                                         *cls.budgets,1)

    def test_clearance_steering_and_order_revalidated(self):
        self.assertGreaterEqual(self.planner.segment_check(self.result.points),.34)
        self.assertIsNotNone(self.planner.feasible(self.result.points,self.anchors))

    def test_search_never_increases_nominal_time(self):
        original=self.planner.estimate(self.seed,*self.budgets)
        self.assertLessEqual(self.report['estimated_lap_seconds'],original+1e-9)

    def test_launch_region_is_preserved(self):
        anchor=self.anchors[np.argmin(np.linalg.norm(self.anchors,axis=1))]
        start=self.seed.project(*anchor)[0]
        for s in np.linspace(start,start+5.9,80):
            point=self.seed.sample(s)
            _,distance=self.result.project(*point)
            self.assertLess(distance,.005)

    def test_infeasible_trials_leave_seed_intact(self):
        with patch.object(self.planner,'feasible',return_value=None):
            result,report=self.planner.refine_multiscale(self.seed,self.anchors,.48,.26,
                                                       *self.budgets,1)
        self.assertIs(result,self.seed)
        self.assertEqual(report['multiscale_moves'],0)

    def test_invalid_search_levels_rejected(self):
        for level in (-1,3,True,1.5):
            with self.assertRaisesRegex(ValueError,'line_search_level'):
                self.planner.build(self.anchors,line_search_level=level)
        with self.assertRaisesRegex(ValueError,'requires optimize_time'):
            self.planner.build(self.anchors,line_search_level=1)


if __name__=='__main__':unittest.main()
