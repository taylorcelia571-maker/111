"""Delayed braking reachability, bounded startup and same-budget tracking."""
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from braking_envelope import delayed_envelope
from time_attack_audit import build,evaluate,BASE,FAST
from race60_audit import rear_path
from predictive_follower import PredictiveFollower


class DelayedBrakingTests(unittest.TestCase):
    def test_every_downstream_cap_has_coasting_and_braking_distance(self):
        raw=np.full(200,7.);raw[45:70]=2.;raw[140:160]=3.
        ds=.05;b=4.;reaction=.25
        speed=delayed_envelope(raw,ds,b,6.,reaction)
        self.assertTrue(np.all(speed<=raw))
        for offset in range(1,len(raw)):
            downstream=np.roll(raw,-offset);faster=speed>downstream+1e-9
            distance=speed*reaction+(speed**2-downstream**2)/(2*b)
            self.assertTrue(np.all(distance[faster]<=offset*ds+1e-8))
        self.assertTrue(np.all(speed**2-np.roll(speed,1)**2<=2*6*ds+1e-9))
        self.assertLess(speed[-1],7.)  # upcoming corner across seam is reserved

    def test_invalid_budgets_are_rejected(self):
        for reaction in (-.1,.6,float('nan')):
            with self.assertRaises(ValueError):delayed_envelope([1,2,3],.1,3,3,reaction)


class TimeAttackTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.old_planner,cls.old_center=build(BASE,'legacy')
        cls.planner,cls.center=build(BASE,'delayed_braking')
        cls.fast_planner,cls.fast_center=build(FAST,'delayed_braking')

    def test_same_budget_improves_first_and_total_model_loops(self):
        old=evaluate(self.old_planner,self.old_center,BASE,.15,'smooth_pursuit','legacy',False)
        new=evaluate(self.planner,self.center,BASE,.15,'race_feedback','delayed_braking',True)
        self.assertTrue(old['completed'],old);self.assertTrue(new['completed'],new)
        self.assertLess(new['seconds'],old['seconds'])
        self.assertLess(new['first_loop_seconds'],old['first_loop_seconds'])
        self.assertLess(new['startup_limit_released_seconds'],old['startup_limit_released_seconds'])

    def test_raised_candidate_for_three_actuator_lags_and_delayed_joints(self):
        for lag,joint_delay in ((.08,0),(.15,2),(.2,2)):
            with self.subTest(lag=lag,joint_delay=joint_delay):
                row=evaluate(self.fast_planner,self.fast_center,FAST,lag,'race_feedback',
                             'delayed_braking',True,joint_delay)
                self.assertTrue(row['completed'],row)
                self.assertLess(row['peak_error'],.15)
                self.assertGreater(row['minimum_center_clearance'],.23)
                self.assertFalse(row['official_result'])

    def follower(self):
        return PredictiveFollower(rear_path(self.center),**BASE,response_delay=.25,
                                  lateral_control='race_feedback',start_blend=True,
                                  startup_release_early=True,steering_feedback_gain=.25,
                                  startup_speed=1.8,startup_acceleration=2,
                                  speed_planning='delayed_braking')

    def test_early_release_requires_progress_and_sustained_measured_alignment(self):
        f=self.follower();f.start(.0081,-.1665,1.5707)
        self.assertTrue(f.launch_limited)
        for _ in range(50):
            f.measured_steer=0;f.measured_steer_age=0
            f.step(.0081,-.1665,1.5707,0,.02)
        self.assertTrue(f.launch_limited)
        self.assertEqual(f.launch_ready,0)
        self.assertLessEqual(f.speed,1.8)
        f.launch_ready=.29;f.stop();f.start(.0081,-.1665,1.5707)
        self.assertEqual(f.launch_ready,0)

    def test_report_identifies_reserved_response_time_and_new_planning_mode(self):
        report=self.follower().plan_report()
        self.assertEqual(report['speed_planning'],'delayed_braking')
        self.assertTrue(report['model_includes_response_preview'])
        self.assertFalse(report['model_includes_startup'])
        self.assertTrue(report['startup_release_early'])
        self.assertAlmostEqual(report['model_five_seconds'],report['response_preview_proxy_five_seconds'])

    def test_startup_diagnostics_identify_why_acceleration_is_limited(self):
        f=self.follower();f.start(.0081,-.1665,1.5707)
        f.measured_steer=0
        f.step(.0081,-.1665,1.5707,0,.02)
        self.assertIn('minimum_progress_or_connector_not_complete',f.last_report['startup_release_blockers'])
        self.assertTrue(f.last_report['acceleration_guard_active'])
        self.assertEqual(f.last_report['active_acceleration'],2)
        self.assertEqual(f.last_report['speed_planning'],'delayed_braking')

    def test_new_controller_uses_requested_acceleration_before_five_meters(self):
        import math
        f=self.follower();x,y,yaw=.0081,-.1665,1.5707
        f.start(x,y,yaw);speed=steer=0
        for tick in range(200):
            f.measured_steer=steer;f.measured_steer_age=0
            v,d=f.step(x,y,yaw,speed,.02)
            if not f.launch_limited:
                self.assertFalse(f.last_report['acceleration_guard_active'])
                self.assertEqual(f.last_report['active_acceleration'],BASE['acceleration'])
                self.assertLess(f.last_report['progress_since_start'],5)
                break
            speed+=(v-speed)*.02/.15;steer+=(d-steer)*.02/.15
            x+=speed*math.cos(yaw)*.02;y+=speed*math.sin(yaw)*.02
            yaw+=speed*math.tan(steer)/.26*.02
        else:self.fail('Measured readiness should release the startup acceleration guard')

    def test_corridor_guard_still_stops_new_controller(self):
        f=self.follower();f.start(.0081,-.1665,1.5707)
        f.measured_steer=0
        with self.assertRaisesRegex(ValueError,'Tracking error exceeds safe corridor'):
            f.step(1.,-.1665,1.5707,1,.02)
        self.assertEqual(f.speed,0)


if __name__=='__main__':unittest.main()
