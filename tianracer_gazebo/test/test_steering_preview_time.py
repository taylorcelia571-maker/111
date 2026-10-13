import copy
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from race_line import ClosedPath
from predictive_follower import PredictiveFollower


class SteeringPreviewTests(unittest.TestCase):
    def test_invalid_tracking_tuning_rejected(self):
        for gain in (-.1,1.1,float('nan')):
            with self.assertRaisesRegex(ValueError,'curvature_feedforward_gain'):
                PredictiveFollower(self.path(),curvature_feedforward_gain=gain)
        for scale in (.7,1.3,float('nan')):
            with self.assertRaisesRegex(ValueError,'pursuit_lookahead_scale'):
                PredictiveFollower(self.path(),pursuit_lookahead_scale=scale)

    def path(self):
        a=np.linspace(0,2*np.pi,401)
        return ClosedPath(np.column_stack((5*np.cos(a),5*np.sin(a))))

    def test_invalid_preview_rejected(self):
        for value in (0,.3,float('nan'),float('inf')):
            with self.assertRaisesRegex(ValueError,'steering_preview_time'):
                PredictiveFollower(self.path(),steering_preview_time=value)

    def test_explicit_default_preserves_commands_and_limits(self):
        p=self.path()
        a=PredictiveFollower(copy.deepcopy(p),lateral_control='smooth_pursuit',response_delay=.25)
        b=PredictiveFollower(copy.deepcopy(p),lateral_control='smooth_pursuit',response_delay=.25,
                             steering_preview_time=.10)
        for f in (a,b):f.start(5,0,np.pi/2)
        for _ in range(10):
            self.assertEqual(a.step(5,0,np.pi/2,0,.02),b.step(5,0,np.pi/2,0,.02))
        self.assertEqual(a.max_error,.15)
        self.assertEqual(a.response_delay,.25)

    def test_preview_does_not_change_planned_speed_envelope(self):
        p=self.path()
        a=PredictiveFollower(copy.deepcopy(p),steering_preview_time=.10,response_delay=.25)
        b=PredictiveFollower(copy.deepcopy(p),steering_preview_time=.15,response_delay=.25)
        np.testing.assert_array_equal(a.path.speeds,b.path.speeds)


if __name__=='__main__':unittest.main()
