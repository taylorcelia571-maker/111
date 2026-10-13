import math
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from acceleration_recovery import AccelerationRecovery
from race_line import ClosedPath
from predictive_follower import PredictiveFollower


class RecoveryTests(unittest.TestCase):
    def test_braking_immediate_but_rebound_bounded(self):
        guard=AccelerationRecovery(.12)
        self.assertEqual(guard.step(3.,'acceleration',.02),3.)
        self.assertEqual(guard.step(1.,'laser',.02),1.)
        self.assertEqual(guard.step(2.,'acceleration',.02),1.)
        self.assertEqual(guard.step(.3,'laser',.02),.3)
        for _ in range(5):self.assertEqual(guard.step(2.,'acceleration',.02),.3)
        for _ in range(2):result=guard.step(2.,'acceleration',.02)
        self.assertEqual(result,2.)

    def test_never_exceeds_requested_cap_and_zero_is_immediate(self):
        g=AccelerationRecovery(.15)
        g.step(3.,'acceleration',.02)
        for requested in (1.1,1.5,.9,1.3,0.,.5,.2):
            self.assertLessEqual(g.step(requested,'laser',.02),requested)
        self.assertEqual(g.step(0.,'laser',.02),0.)

    def test_disabled_is_identity_and_reset_removes_old_hold(self):
        g=AccelerationRecovery(0.)
        for requested in (3.,1.,2.,0.):self.assertEqual(g.step(requested,'laser',.02),requested)
        g=AccelerationRecovery(.12);g.step(3.,'acceleration',.02);g.step(1.,'laser',.02)
        g.reset();self.assertEqual(g.step(2.,'acceleration',.02),2.)

    def test_invalid_input_rejected(self):
        for hold in (-.1,.5,float('nan')):
            with self.assertRaises(ValueError):AccelerationRecovery(hold)
        for speed,dt in ((float('nan'),.02),(-1,.02),(1,0),(1,.4)):
            with self.assertRaises(ValueError):AccelerationRecovery(.12).step(speed,'laser',dt)


class SmoothEarlyStartupTests(unittest.TestCase):
    def test_curvature_feedforward_is_continuous_at_sample_boundary(self):
        values=[]
        for offset in (-1e-7,1e-7):
            f=self.follower();f.start_blend=False
            f.path.curvature[:]=.1;f.path.curvature[10]=.3
            s=10*f.path.ds+offset;x,y=f.path.sample(s)
            tangent=f.path.sample(s+.08)-f.path.sample(s-.08)
            yaw=math.atan2(tangent[1],tangent[0])
            f.start(x,y,yaw);f.measured_steer=.026;f.measured_steer_age=0
            f.step(x,y,yaw,1.,.02)
            values.append(f.last_report['feedforward_curvature'])
        self.assertLess(abs(values[0]-values[1]),1e-5)

    def follower(self):
        a=np.linspace(0,2*np.pi,801)
        p=ClosedPath(np.column_stack((10*np.cos(a),10*np.sin(a))))
        return PredictiveFollower(p,lateral_control='smooth_pursuit',start_blend=True,
            startup_release_early=True,startup_speed=1.8,startup_acceleration=2.,
            acceleration=6.6,steering_feedback_gain=.25,response_delay=.25)

    def test_aligned_connector_releases_after_distance_and_sustained_readiness(self):
        f=self.follower();f.start(10,0,math.pi/2)
        self.assertTrue(f.launch_limited)
        for tick in range(100):
            s=tick*.025;x,y=f.sample_tracking(s)
            tangent=f.sample_tracking(s+.08)-f.sample_tracking(s-.08)
            yaw=math.atan2(tangent[1],tangent[0])
            f.measured_steer=math.atan(f.wheelbase*.1);f.measured_steer_age=0
            f.step(x,y,yaw,1.25,.02)
            if not f.launch_limited:
                self.assertGreaterEqual(f.progress-f.origin,.6)
                self.assertGreaterEqual(f.launch_ready,.3)
                self.assertLess(f.progress-f.origin,2.5)
                self.assertEqual(f.last_report['active_acceleration'],6.6)
                break
        else:self.fail('Aligned smooth controller should support early release')

    def test_stationary_or_misaligned_feedback_never_releases(self):
        f=self.follower();f.start(10,0,math.pi/2)
        for _ in range(30):
            f.measured_steer=0;f.measured_steer_age=0
            f.step(10,0,math.pi/2,0,.02)
        self.assertTrue(f.launch_limited)
        f.launch_ready=.29
        s=.8;x,y=f.sample_tracking(s)
        f.progress=f.origin+s
        f.measured_steer=.4
        f.step(x,y,math.pi/2,1.25,.02)
        self.assertTrue(f.launch_limited)
        self.assertEqual(f.launch_ready,0.)


if __name__=='__main__':unittest.main()
