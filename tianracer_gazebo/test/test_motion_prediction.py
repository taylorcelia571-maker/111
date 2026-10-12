import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from motion_prediction import predict_rear_pose


class PredictionTests(unittest.TestCase):
    def test_straight_delay_with_lateral_velocity(self):
        x,y,yaw=predict_rear_pose(1,2,0,3,0,.05,.2)
        self.assertAlmostEqual(x,1.15)
        self.assertAlmostEqual(y,2.01)
        self.assertEqual(yaw,0)

    def test_circular_arc_and_inverse(self):
        pose=predict_rear_pose(0,0,0,2,1,.05)
        self.assertAlmostEqual(pose[0],2*math.sin(.05))
        self.assertAlmostEqual(pose[1],2*(1-math.cos(.05)))
        restored=predict_rear_pose(*pose,-2,-1,.05)
        for value in restored:self.assertAlmostEqual(value,0)

    def test_long_delay_and_nonfinite_inputs_rejected(self):
        for age in [-.01,.081,float('nan')]:
            with self.assertRaises(ValueError):predict_rear_pose(0,0,0,2,1,age)
        with self.assertRaises(ValueError):predict_rear_pose(0,0,0,2,float('nan'),.02)


if __name__=='__main__':unittest.main()
