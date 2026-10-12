"""Delayed bicycle response regressions; no Gazebo/official-result claim."""
import math
import unittest
import xml.etree.ElementTree as ET
import numpy as np
import test_planned_loop as course
from test_planned_loop import rear_path
from predictive_follower import PredictiveFollower


class FinishTrackingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        course.PlannedLoopTests.setUpClass()
        cls.center = course.PlannedLoopTests.center
        cls.planner = course.PlannedLoopTests.planner
        launch=ET.parse(course.PACKAGE/'launch/finish_race.launch').getroot()
        cls.budgets={item.get('name'):float(item.get('default')) for item in launch.findall('arg')}

    def test_five_laps_with_delayed_speed_and_steering(self):
        # Test a range of hypothetical delays; .25s is an allowance, not a
        # measured plant identification or a guarantee about Gazebo friction.
        for delay in (.08, .15, .25):
            with self.subTest(delay=delay):
                p = rear_path(self.center)
                f = PredictiveFollower(p, **self.budgets, yaw_rate=2.5,
                                       fast_start=True,lateral_control='curvature_feedback')
                x,y,yaw = .0081,-.1494,1.5709
                f.start(x,y,yaw)
                speed=steer=0
                seams=0
                for tick in range(10000):
                    f.measured_steer=steer
                    v,d = f.step(x,y,yaw,speed,.02)
                    self.assertGreaterEqual(v,0)
                    speed += (v-speed)*.02/delay
                    steer += (d-steer)*.02/delay
                    x += speed*math.cos(yaw)*.02
                    y += speed*math.sin(yaw)*.02
                    yaw += speed*math.tan(steer)/.26*.02
                    self.assertLess(f.error,.12)
                    clearance=self.planner.clearance_at([[x+.13*math.cos(yaw),y+.13*math.sin(yaw)]])[0]
                    self.assertGreater(clearance,.23)
                    lap=int((f.progress-f.origin)/p.length)
                    if lap>seams:
                        # A slow actuator can require a tracking correction,
                        # but crossing a seam must not re-enter departure or stop.
                        self.assertGreater(v,.05)
                        self.assertFalse(f.departure)
                        seams=lap
                    if seams==5: break
                self.assertEqual(seams,5,'Five model loops must complete')

    def test_steering_feedback_restricts_speed_until_joints_align(self):
        p=rear_path(self.center)
        i=int(np.argmin(abs(p.curvature)));s=i*p.ds;x,y=p.sample(s)
        t=p.sample(s+.08)-p.sample(s-.08);yaw=math.atan2(t[1],t[0])
        results=[]
        for actual in (0,.25):
            f=PredictiveFollower(rear_path(self.center),**self.budgets,
                                 lateral_control='curvature_feedback')
            f.start(x,y,yaw);f.departure=False;f.speed=2.2;f.measured_steer=actual
            v,_=f.step(x,y,yaw,2.2,.02);results.append(v)
        self.assertGreater(results[0],results[1])
        self.assertLess(results[1],.6)

    def test_continuous_steering_across_old_recovery_switch(self):
        p=rear_path(self.center);i=int(np.argmin(abs(p.curvature)));s=i*p.ds
        q=p.sample(s);t=p.sample(s+.08)-p.sample(s-.08);heading=math.atan2(t[1],t[0])
        desired=[]
        for error in (.059,.061):
            f=PredictiveFollower(rear_path(self.center),**self.budgets,
                                 lateral_control='curvature_feedback')
            x=q[0]-error*math.sin(heading);y=q[1]+error*math.cos(heading)
            f.start(x,y,heading+.1);f.departure=False
            f.step(x,y,heading+.1,2.2,.02)
            desired.append(f.last_report['desired_steer'])
        self.assertLess(abs(desired[0]-desired[1]),.005)

    def test_localization_jitter_with_delayed_actuators(self):
        p=rear_path(self.center)
        f=PredictiveFollower(p,**self.budgets,fast_start=True,
                             lateral_control='curvature_feedback')
        x,y,yaw=.0081,-.1494,1.5709;f.start(x,y,yaw);speed=steer=0
        for tick in range(4000):
            f.measured_steer=steer
            # Synthetic millimeter position jitter and .005rad heading jitter.
            v,d=f.step(x+.003*math.sin(tick*.31),y+.003*math.cos(tick*.27),
                       yaw+.005*math.sin(tick*.33),speed,.02)
            speed+=(v-speed)*.02/.15;steer+=(d-steer)*.02/.15
            x+=speed*math.cos(yaw)*.02;y+=speed*math.sin(yaw)*.02
            yaw+=speed*math.tan(steer)/.26*.02
            self.assertLess(p.project(x,y)[1],.12)
            self.assertGreater(self.planner.clearance_at([[x+.13*math.cos(yaw),y+.13*math.sin(yaw)]])[0],.23)
            if f.progress-f.origin>=p.length:break
        self.assertGreaterEqual(f.progress-f.origin,p.length)

    def test_measured_speed_advances_braking_before_corner(self):
        p=rear_path(self.center)
        baseline=PredictiveFollower(p,max_speed=4)
        count=int(math.ceil(4*.25/p.ds))
        losses=np.array([p.speeds[i]-min(p.speeds[(i+np.arange(count+1))%p.n]) for i in range(p.n)])
        i=int(np.argmax(losses)); self.assertGreater(losses[i],.3)
        s=i*p.ds; x,y=p.sample(s)
        t=p.sample(s+.08)-p.sample(s-.08); yaw=math.atan2(t[1],t[0])
        delayed=PredictiveFollower(rear_path(self.center),max_speed=4,response_delay=.25)
        for f in (baseline,delayed):
            f.start(x,y,yaw);f.departure=False;f.speed=4
        v0,_=baseline.step(x,y,yaw,4,.02)
        v1,_=delayed.step(x,y,yaw,4,.02)
        self.assertLess(v1,v0)
        self.assertAlmostEqual(delayed.last_report['preview_distance'],1)

    def test_invalid_response_allowance_rejected(self):
        for delay in (-.01,.51,float('nan'),float('inf')):
            with self.assertRaises(ValueError):
                PredictiveFollower(rear_path(self.center),response_delay=delay)

    def test_ceiling_sensitivity_above_peak_and_no_state_mutation(self):
        p=rear_path(self.center)
        f=PredictiveFollower(p,max_speed=8,lateral_accel=6.5,acceleration=5,yaw_rate=3.8)
        points=p.points.copy();speeds=p.speeds.copy()
        report=f.plan_report()
        self.assertFalse(report['speed_ceiling_binding'])
        self.assertLess(report['peak_planned_speed'],8)
        self.assertAlmostEqual(report['same_route_parameter_sensitivity']['max_speed_plus_1']['model_seconds_saved'],0)
        self.assertGreater(report['same_route_parameter_sensitivity']['lateral_plus_10_percent']['model_seconds_saved'],0)
        np.testing.assert_array_equal(p.points,points)
        np.testing.assert_array_equal(p.speeds,speeds)

    def test_ceiling_sensitivity_detects_a_real_straight_speed_limit(self):
        f=PredictiveFollower(rear_path(self.center),max_speed=1.5,lateral_accel=4)
        report=f.plan_report()
        self.assertTrue(report['speed_ceiling_binding'])
        self.assertGreater(report['same_route_parameter_sensitivity']['max_speed_plus_1']['model_seconds_saved'],10)


if __name__ == '__main__': unittest.main()
