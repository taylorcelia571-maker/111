"""High-budget pursuit regressions on the actual map, not Gazebo results."""
import math
import unittest
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


if __name__=='__main__':unittest.main()
