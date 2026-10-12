import math
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from motion_metrics import summarize
from wheel_kinematics import wheel_commands


class MetricsTests(unittest.TestCase):
    def test_distinguish_planning_slowdown_from_drive_deficit(self):
        t=np.arange(0,5,.02)
        samples={'cmd_vel':[dict(t=float(x),speed=2 if x<2.5 else 1,steer=0 if x<2.5 else .2) for x in t],
                 'odom':[dict(t=float(x),speed=1.5 if x<2.5 else .6) for x in t]}
        s=summarize(samples,2)
        self.assertAlmostEqual(s['straight']['command_speed_mean_m_s'],2)
        self.assertAlmostEqual(s['corner']['command_speed_mean_m_s'],1)
        self.assertAlmostEqual(s['corner']['speed_deficit_mean_m_s'],.4)

    def test_speed_guard_is_not_misreported_as_actuator_deficit(self):
        t=np.arange(0,3,.02)
        samples={'cmd_vel':[dict(t=float(x),speed=2.2,steer=.2) for x in t],
                 'ackermann':[dict(t=float(x),speed=1.6,steer=.2) for x in t],
                 'odom':[dict(t=float(x),speed=1.6) for x in t]}
        report=summarize(samples,2.2)['corner']
        self.assertAlmostEqual(report['speed_deficit_mean_m_s'],0)
        self.assertAlmostEqual(report['planner_to_actuator_reduction_mean_m_s'],.6)

    def test_known_steering_delay_estimation(self):
        t=np.arange(0,5,.02)
        sample={'cmd_vel':[], 'ackermann':[], 'joints':[]}
        for x in t:
            steer=.2*math.sin(3*x)
            sample['cmd_vel'].append(dict(t=float(x),speed=1,steer=steer))
            sample['ackermann'].append(dict(t=float(x),steer=steer))
            c=wheel_commands(1,.2*math.sin(3*(x-.1)),True)
            sample['joints'].append(dict(t=float(x),left=c['left'],right=c['right']))
        self.assertAlmostEqual(summarize(sample)['steering_lag_estimate_s'],.1,places=2)

    def test_clock_reset_invalidates_statistics(self):
        s=summarize({'cmd_vel':[dict(t=2,speed=1,steer=0),dict(t=1,speed=1,steer=0)]})
        self.assertIn('Simulation clock reset',s['insufficient_data'])

    def test_insufficient_data_report(self):
        self.assertIn('insufficient_data',summarize({}))


if __name__=='__main__':unittest.main()
