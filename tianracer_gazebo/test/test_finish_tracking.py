"""Delayed bicycle response regressions; no Gazebo/official-result claim."""
import math
import unittest
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

    def test_five_laps_with_delayed_speed_and_steering(self):
        # Test a range of hypothetical delays; .25s is an allowance, not a
        # measured plant identification or a guarantee about Gazebo friction.
        for delay in (.08, .15, .25):
            with self.subTest(delay=delay):
                p = rear_path(self.center)
                f = PredictiveFollower(p, max_speed=2, lateral_accel=3,
                                       acceleration=1, braking=2, yaw_rate=2.5,
                                       response_delay=.25, fast_start=True)
                x,y,yaw = .0081,-.1494,1.5709
                f.start(x,y,yaw)
                speed=steer=0
                seams=0
                for tick in range(10000):
                    v,d = f.step(x,y,yaw,speed,.02)
                    self.assertGreaterEqual(v,0)
                    speed += (v-speed)*.02/delay
                    steer += (d-steer)*.02/delay
                    x += speed*math.cos(yaw)*.02
                    y += speed*math.sin(yaw)*.02
                    yaw += speed*math.tan(steer)/.26*.02
                    self.assertLess(f.error,.15)
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


if __name__ == '__main__': unittest.main()
