"""High-budget pursuit regressions on the actual map, not Gazebo results."""
import math
import unittest
import numpy as np
import test_planned_loop as course
from predictive_follower import PredictiveFollower


class SmoothPursuitTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        course.PlannedLoopTests.setUpClass()
        cls.budgets=dict(max_speed=8,lateral_accel=6.5,acceleration=5,braking=3,
                         yaw_rate=3.8,steering_rate=2)
        cls.planner=course.PlannedLoopTests.planner
        cls.center=cls.planner.build(course.PlannedLoopTests.anchors,optimize_time=True,**cls.budgets)

    def follower(self):
        return PredictiveFollower(course.rear_path(self.center),**self.budgets,
                                  response_delay=.25,fast_start=True,lateral_control='smooth_pursuit')

    def test_five_laps_with_command_fallback_and_real_steering_feedback(self):
        # 200ms lag without steering feedback can still exceed the corridor;
        # that unsupported condition must retain a protective stop.
        for lag,feedback in ((.08,False),(.12,False),(.2,True)):
            with self.subTest(lag=lag,feedback=feedback):
                f=self.follower();x,y,yaw=.0081,-.1494,1.5709
                f.start(x,y,yaw);speed=steer=0;seams=0
                for tick in range(8000):
                    if feedback:f.measured_steer=steer
                    v,d=f.step(x,y,yaw,speed,.02)
                    self.assertGreaterEqual(v,0)
                    speed+=(v-speed)*.02/lag;steer+=(d-steer)*.02/lag
                    x+=speed*math.cos(yaw)*.02;y+=speed*math.sin(yaw)*.02
                    yaw+=speed*math.tan(steer)/.26*.02
                    self.assertLess(f.error,.15)
                    clearance=self.planner.clearance_at([[x+.13*math.cos(yaw),y+.13*math.sin(yaw)]])[0]
                    self.assertGreater(clearance,.23)
                    lap=int((f.progress-f.origin)/f.path.length)
                    if lap>seams:
                        self.assertFalse(f.departure)
                        self.assertGreater(v,.05)
                        seams=lap
                    if seams==5:break
                self.assertEqual(seams,5)

    def test_eight_degree_heading_error_does_not_release_departure(self):
        f=self.follower();s=25;q=f.path.sample(s)
        t=f.path.sample(s+.08)-f.path.sample(s-.08);heading=math.atan2(t[1],t[0])
        f.start(*q,heading+.14)
        for _ in range(25):f.step(*q,heading+.14,0,.02)
        self.assertTrue(f.departure)
        self.assertLessEqual(f.speed,1)

    def test_recovery_threshold_does_not_jump_preview_or_desired_steer(self):
        reports=[]
        for error in (.0599,.0601):
            f=self.follower();s=25;q=f.path.sample(s)
            t=f.path.sample(s+.08)-f.path.sample(s-.08);heading=math.atan2(t[1],t[0])
            x=q[0]-error*math.sin(heading);y=q[1]+error*math.cos(heading)
            f.start(x,y,heading+.10);f.departure=False;f.speed=3
            f.step(x,y,heading+.10,3,.02)
            reports.append(f.last_report)
        self.assertLess(abs(reports[0]['preview_length']-reports[1]['preview_length']),.003)
        self.assertLess(abs(reports[0]['desired_steer']-reports[1]['desired_steer']),.005)
        self.assertLess(abs(reports[0]['command_speed']-reports[1]['command_speed']),.02)

    def test_corridor_stop_remains_enabled(self):
        f=self.follower();s=25;q=f.path.sample(s)
        t=f.path.sample(s+.08)-f.path.sample(s-.08);heading=math.atan2(t[1],t[0])
        f.start(*q,heading)
        with self.assertRaisesRegex(ValueError,'Tracking error exceeds safe corridor'):
            f.step(q[0]-.2*math.sin(heading),q[1]+.2*math.cos(heading),heading,2,.02)
        self.assertEqual(f.speed,0)

    def test_connector_tracks_actual_start_not_nominal_offset(self):
        f=PredictiveFollower(course.rear_path(self.center),**self.budgets,
                             start_blend=True,fast_start=True,lateral_control='smooth_pursuit')
        x,y,yaw=.0081,-.1577,1.5709;f.start(x,y,yaw)
        self.assertEqual(f.start_diagnostics['mode'],'connector')
        self.assertFalse(f.departure)
        f.step(x,y,yaw,0,.02)
        self.assertGreater(f.error,.06)  # nominal guard remains based on original line
        self.assertLess(f.last_report['control_tracking_error'],1e-6)

    def test_start_connector_vehicle_centers_have_required_map_clearance(self):
        f=PredictiveFollower(course.rear_path(self.center),**self.budgets,
                             start_blend=True,lateral_control='smooth_pursuit')
        f.start(.0081,-.1577,1.5709)
        centers=[]
        for distance in np.linspace(f.origin,f.origin+f.connector_length,100):
            point=f.sample_tracking(distance)
            tangent=f.sample_tracking(distance+.02)-f.sample_tracking(distance-.02)
            tangent/=np.linalg.norm(tangent)
            centers.append(point+.13*tangent)
        self.assertGreaterEqual(float(self.planner.clearance_at(centers).min()),.26)

    def test_logged_start_cannot_request_four_meters_per_second_in_first_second(self):
        f=PredictiveFollower(course.rear_path(self.center),**self.budgets,start_blend=True,
                             response_delay=.25,lateral_control='smooth_pursuit',fast_start=True)
        x,y,yaw=.0081,-.1591,1.5707;f.start(x,y,yaw);speed=steer=0
        for _ in range(50):
            f.measured_steer=steer
            v,d=f.step(x,y,yaw,speed,.02)
            self.assertTrue(f.launch_limited)
            self.assertLessEqual(v,1.2)
            self.assertLessEqual(f.last_report['active_acceleration'],1.5)
            speed+=(v-speed)*.02/.12;steer+=(d-steer)*.02/.12
            x+=speed*math.cos(yaw)*.02;y+=speed*math.sin(yaw)*.02
            yaw+=speed*math.tan(steer)/.26*.02
        self.assertLessEqual(speed,1.2)

    def test_launch_limit_resets_on_restart(self):
        f=PredictiveFollower(course.rear_path(self.center),**self.budgets,start_blend=True,
                             lateral_control='smooth_pursuit')
        f.start(.0081,-.1591,1.5707)
        f.launch_ready=.28
        f.stop();f.start(.0081,-.1591,1.5707)
        self.assertEqual(f.launch_ready,0)
        self.assertTrue(f.launch_limited)

    def test_misaligned_start_uses_departure_instead_of_connector(self):
        f=PredictiveFollower(course.rear_path(self.center),**self.budgets,start_blend=True,
                             lateral_control='smooth_pursuit')
        f.start(.0081,-.1591,1.7107)
        self.assertIsNone(f.connector_offset)
        self.assertTrue(f.departure)

    def test_fast_connector_five_laps_keeps_wall_and_tracking_guards(self):
        for lag in (.08,.12,.2):
            with self.subTest(lag=lag):
                f=PredictiveFollower(course.rear_path(self.center),**self.budgets,start_blend=True,
                                     fast_start=True,response_delay=.15,lateral_control='smooth_pursuit')
                x,y,yaw=.0081,-.1577,1.5709;f.start(x,y,yaw);speed=steer=0
                for tick in range(8000):
                    f.measured_steer=steer
                    v,d=f.step(x,y,yaw,speed,.02)
                    speed+=(v-speed)*.02/lag;steer+=(d-steer)*.02/lag
                    x+=speed*math.cos(yaw)*.02;y+=speed*math.sin(yaw)*.02
                    yaw+=speed*math.tan(steer)/.26*.02
                    self.assertLess(f.error,.15)
                    self.assertGreater(self.planner.clearance_at([[x+.13*math.cos(yaw),y+.13*math.sin(yaw)]])[0],.23)
                    if f.progress-f.origin>=5*f.path.length:break
                self.assertGreaterEqual(f.progress-f.origin,5*f.path.length)


if __name__=='__main__':unittest.main()
