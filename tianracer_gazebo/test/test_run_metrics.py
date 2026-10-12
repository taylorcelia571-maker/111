"""Summary accuracy and interval attribution, without a ROS clock."""
import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from run_metrics import RunMetrics


class MetricsTests(unittest.TestCase):
    def test_time_weighting_and_reference_loop_are_not_official_laps(self):
        m=RunMetrics()
        m.observe(0,0,.5,.07,'departure',True,0,0)
        m.observe(1,1,1,.10,'acceleration',False,1,.5)
        self.assertTrue(m.observe(3,3,2,.05,'planned_envelope',False,0,1.5))
        r=m.report()
        self.assertFalse(r['official_result'])
        self.assertEqual(r['reference_loop_seconds'],[3])
        self.assertEqual(r['peak_measured_speed'],3)
        self.assertEqual(r['peak_reference_tracking_error'],.10)
        self.assertEqual(r['time_by_state']['phase:departure']['seconds'],1)
        self.assertEqual(r['time_by_state']['acceleration']['seconds'],2)
        self.assertEqual(r['time_by_state']['acceleration']['mean_measured_speed'],2)
        self.assertEqual(r['time_by_state']['acceleration']['mean_command_speed'],1)
        self.assertEqual(r['time_by_state']['road:curve']['seconds'],2)

    def test_zero_time_does_not_double_count_a_paused_clock(self):
        m=RunMetrics();m.observe(0,0,0,0,'idle',True,0,0)
        m.observe(0,0,0,0,'idle',True,0,0)
        self.assertEqual(m.report()['observed_simulation_seconds'],0)
        self.assertEqual(m.report()['time_by_state']['idle']['seconds'],0)

    def test_invalid_or_backwards_samples_are_rejected(self):
        m=RunMetrics();m.observe(1,1,1,0,'tracking',False,0,0)
        with self.assertRaises(ValueError):m.observe(.5,1,1,0,'tracking',False,0,0)
        with self.assertRaises(ValueError):m.observe(2,math.nan,1,0,'tracking',False,0,0)


if __name__=='__main__':unittest.main()
