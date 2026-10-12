"""High-budget rapid route retains wall reserve; model results are not official."""
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from race60_audit import load_course, simulate


class RapidWallTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.planner,cls.anchors=load_course(Path(__file__).resolve().parents[1])
        cls.planner.margin=.34
        cls.planner.allowed=cls.planner.clearance>=.34
        cls.budgets=dict(max_speed=10,lateral_accel=6.5,acceleration=5.5,
                         braking=3.,yaw_rate=3.8,steering_rate=2.)
        cls.base=cls.planner.build(cls.anchors,optimize_time=True,response_delay=.25,**cls.budgets)
        cls.refined=cls.planner.build(cls.anchors,optimize_time=True,refine_line=True,
                                     response_delay=.25,**cls.budgets)

    def test_rapid_and_refined_five_model_loops_with_fresh_joint_feedback(self):
        for center in (self.base,self.refined):
            self.assertGreaterEqual(self.planner.segment_check(center.points),.34)
            for lag in (.08,.12,.2):
                with self.subTest(lag=lag,refined=center is self.refined):
                    report=simulate(center,self.planner,self.budgets,.25,lag,lateral_control='smooth_pursuit')
                    self.assertTrue(report['completed'],report)
                    self.assertGreater(report['minimum_center_clearance'],.30)
                    self.assertLess(report['peak_error'],.15)

    def test_refinement_gains_time_without_using_the_old_thin_wall_reserve(self):
        baseline=simulate(self.base,self.planner,self.budgets,.25,.12,lateral_control='smooth_pursuit')
        trial=simulate(self.refined,self.planner,self.budgets,.25,.12,lateral_control='smooth_pursuit')
        self.assertTrue(baseline['completed'])
        self.assertTrue(trial['completed'])
        self.assertLess(trial['seconds'],baseline['seconds'])
        self.assertGreater(trial['minimum_center_clearance'],.30)


if __name__=='__main__':unittest.main()
