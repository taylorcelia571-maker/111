"""Bounded public-joint feedback experiments; not Gazebo or official results."""
from collections import deque
import math
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from race60_audit import load_course,rear_path
from predictive_follower import PredictiveFollower


class SteeringCompensationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.planner,anchors=load_course(Path(__file__).resolve().parents[1])
        cls.planner.margin=.34;cls.planner.allowed=cls.planner.clearance>=.34
        cls.budgets=dict(max_speed=10,lateral_accel=6.5,acceleration=5.5,
                         braking=3,yaw_rate=3.8,steering_rate=2)
        cls.center=cls.planner.build(anchors,optimize_time=True,response_delay=.25,**cls.budgets)

    def run_model(self,gain,lag,joint_delay=0):
        f=PredictiveFollower(rear_path(self.center),response_delay=.25,fast_start=True,
                             lateral_control='smooth_pursuit',steering_feedback_gain=gain,**self.budgets)
        x,y,yaw=.0081,-.1546,1.5689;f.start(x,y,yaw);speed=steer=0
        history=deque([0.]*(joint_delay+1),maxlen=joint_delay+1)
        first=None;minimum=1.;error=0.
        for tick in range(10000):
            f.measured_steer=history[0];f.measured_steer_age=joint_delay*.02
            previous=f.steer
            v,d=f.step(x,y,yaw,speed,.02)
            self.assertGreaterEqual(v,0)
            self.assertLessEqual(abs(d),.48)
            self.assertLessEqual(abs(d-previous),.040001)
            self.assertLessEqual(v*v*f.last_report['constraint_curvature'],6.500001)
            speed+=(v-speed)*.02/lag;steer+=(d-steer)*.02/lag
            history.append(steer)
            x+=speed*math.cos(yaw)*.02;y+=speed*math.sin(yaw)*.02
            yaw+=speed*math.tan(steer)/.26*.02
            error=max(error,f.error)
            minimum=min(minimum,float(self.planner.clearance_at([[x+.13*math.cos(yaw),y+.13*math.sin(yaw)]])[0]))
            loops=(f.progress-f.origin)/f.path.length
            if first is None and loops>=1:first=(tick+1)*.02
            if loops>=5:return dict(seconds=(tick+1)*.02,first=first,minimum=minimum,error=error)
        self.fail('Model did not finish five reference loops')

    def test_delayed_actuator_tracking_improves_without_relaxing_wall_guard(self):
        baseline=self.run_model(0,.2)
        candidate=self.run_model(.25,.2)
        self.assertLess(candidate['seconds'],baseline['seconds'])
        self.assertLess(candidate['first'],baseline['first'])
        self.assertGreater(candidate['minimum'],.30)
        self.assertLess(candidate['error'],.15)

    def test_five_loops_with_fast_actuator_and_delayed_public_joints(self):
        for lag,joint_delay in ((.08,0),(.15,2),(.2,4)):
            with self.subTest(lag=lag,joint_delay=joint_delay):
                row=self.run_model(.25,lag,joint_delay)
                self.assertGreater(row['minimum'],.30)
                self.assertLess(row['error'],.15)

    def test_old_joint_sample_disables_extra_compensation(self):
        f=PredictiveFollower(rear_path(self.center),lateral_control='smooth_pursuit',steering_feedback_gain=.25)
        f.start(.0081,-.1546,1.5689);f.measured_steer=.2;f.measured_steer_age=.10
        f.step(.0081,-.1546,1.5689,0,.02)
        self.assertEqual(f.last_report['effective_steering_feedback_gain'],0)
        self.assertEqual(f.last_report['actuator_target_steer'],f.last_report['desired_steer'])

    def test_invalid_gain_rejected(self):
        for gain in (-.1,.6,float('nan')):
            with self.assertRaises(ValueError):
                PredictiveFollower(rear_path(self.center),steering_feedback_gain=gain)

    def test_jitter_at_freshness_boundary_does_not_switch_full_compensation(self):
        reports=[]
        for age in (.0799,.0801):
            f=PredictiveFollower(rear_path(self.center),lateral_control='smooth_pursuit',steering_feedback_gain=.25)
            f.start(.0081,-.1546,1.5689);f.measured_steer=.2;f.measured_steer_age=age
            f.step(.0081,-.1546,1.5689,0,.02);reports.append(f.last_report)
        self.assertLess(abs(reports[0]['actuator_target_steer']-reports[1]['actuator_target_steer']),.001)


if __name__=='__main__':unittest.main()
