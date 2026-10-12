import math
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from corner_speed import CornerSpeed, path_preview


class CornerSpeedTests(unittest.TestCase):
    def settle(self, control, speed=2.2, steer=0, actual=0, preview=None):
        if preview is None: preview=[(0,0),(3,0)]
        for _ in range(100): out=control.step(speed,steer,actual,speed,preview,.02)
        return out

    def test_straight_reaches_higher_speed_without_overriding_planner(self):
        c=CornerSpeed(2.2,1.6)
        self.assertAlmostEqual(self.settle(c),2.2)
        self.assertEqual(c.step(.8,0,0,2.2,[(0,0)],.02),.8)

    def test_current_bend_has_distinct_speed_cap(self):
        c=CornerSpeed(2.2,1.6)
        self.settle(c)
        v=c.step(2.2,.3,.3,2.2,[(0,1.2)],.02)
        self.assertLessEqual(v,1.6)

    def test_actual_steering_prevents_premature_exit_acceleration(self):
        c=CornerSpeed(2.2,1.6)
        self.assertLessEqual(self.settle(c,steer=0,actual=.3),1.6)
        self.assertGreater(self.settle(c,steer=0,actual=0),1.6)

    def test_preview_slows_before_command_steering_changes(self):
        c=CornerSpeed(2.2,1.6)
        self.settle(c)
        near=c.step(2.2,0,0,2.2,[(.7,2)],.02)
        self.assertLess(near,2.2)
        self.assertEqual(c.reason,'preview_turn')

    def test_stop_invalid_and_reverse(self):
        c=CornerSpeed(2.2,1.6)
        self.settle(c)
        self.assertEqual(c.step(0,.2,.2,2,[(0,1)],.02),0)
        self.assertEqual(c.step(float('nan'),0,0,0,[],.02),0)
        self.settle(c)
        self.assertEqual(c.step(-1,0,0,2,[(0,0)],.02),0)
        self.assertEqual(c.step(-1,0,0,1,[(0,0)],.02),0)
        self.assertLess(c.step(-1,0,0,0,[(0,0)],.02),0)

    def test_missing_feedback_limits_speed_without_assuming_straight(self):
        c=CornerSpeed(2.2,1.6)
        for _ in range(100): v=c.step(2.2,0,None,2,None,.02)
        self.assertLessEqual(v,1.6)
        self.assertEqual(c.reason,'feedback_fallback')

    def test_acceleration_is_bounded_and_steering_is_not_filtered(self):
        c=CornerSpeed(2.2,1.6)
        previous=0
        for _ in range(50):
            v=c.step(2.2,0,0,1,[(0,0)],.02)
            self.assertLessEqual(v-previous,.02*c.acceleration+1e-9)
            previous=v

    def test_preview_geometry(self):
        t=np.linspace(0,math.pi/2,100)
        p=np.column_stack([2*np.sin(t),2*(1-np.cos(t))])
        preview=path_preview(p)
        self.assertTrue(preview)
        self.assertAlmostEqual(float(np.median([k for _,k in preview])),.5,places=2)
        self.assertIsNone(path_preview([[0,0],[1,0]]))
        self.assertIsNone(path_preview([[0,0],[0,0],[0,0]]))


if __name__=='__main__': unittest.main()
