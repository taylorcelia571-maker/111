"""Offline transport-adapter tests using fake messages/time, not a ROS runtime."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
from corner_speed import CornerSpeed  # Load NumPy before temporary sys.modules patches.


class Stamp:
    now_value = 0.0

    def __init__(self, value=0.0):
        self.value = value

    def __sub__(self, other):
        return Stamp(self.value - other.value)

    def to_sec(self):
        return self.value

    @classmethod
    def now(cls):
        return cls(cls.now_value)


def ackermann():
    return SimpleNamespace(header=SimpleNamespace(), drive=SimpleNamespace())


class ConverterTests(unittest.TestCase):
    def test_immediate_raw_command_cannot_bypass_clock_reset_guard(self):
        self.params={'~race_profile':False,'~command_watchdog':True}
        converter=self.module.Converter()
        Stamp.now_value=1.;self.command(converter,3.,.1)
        Stamp.now_value=1.02;converter.tick(None)
        Stamp.now_value=.5;self.command(converter,3.,.1)
        self.assertEqual(self.latest(converter).speed,0.)
        self.assertIsNone(converter.received)
        Stamp.now_value=.52;converter.tick(None)
        self.assertEqual(self.latest(converter).speed,0.)
        Stamp.now_value=.54;self.command(converter,3.,.1)
        self.assertEqual(self.latest(converter).speed,3.)

    def test_raw_watchdog_forwards_before_tick_without_duplicate_live_replay(self):
        self.params={'~race_profile':False,'~command_watchdog':True}
        converter=self.module.Converter()
        Stamp.now_value=.011
        self.command(converter,3.2,-.18)
        self.assertEqual(self.latest(converter).speed,3.2)
        self.assertEqual(self.latest(converter).steering_angle,-.18)
        count=converter.pub.publish.call_count
        Stamp.now_value=.02;converter.tick(None)
        self.assertEqual(converter.pub.publish.call_count,count)
        converter.tick(None)  # repeated simulation clock is not an expired command
        self.assertEqual(converter.pub.publish.call_count,count)
        self.assertEqual(self.latest(converter).speed,3.2)
        Stamp.now_value=.6;converter.tick(None)
        self.assertEqual(self.latest(converter).speed,0.)

    def setUp(self):
        Stamp.now_value = 0.0
        self.params = {'~race_profile': True}
        self.ros = Mock()
        self.ros.get_param.side_effect = lambda key, default=None: self.params.get(key, default)
        self.ros.Time = Stamp
        self.ros.Duration = Stamp
        modules = {'rospy': self.ros,
                   'ackermann_msgs.msg': SimpleNamespace(AckermannDriveStamped=ackermann),
                   'geometry_msgs.msg': SimpleNamespace(Twist=object)}
        with patch.dict(sys.modules, modules):
            spec = importlib.util.spec_from_file_location('race_transform_test', SCRIPTS / 'transform.py')
            self.module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(self.module)

    def command(self, converter, speed=2.4, steer=0.2):
        converter.callback(SimpleNamespace(linear=SimpleNamespace(x=speed),
                                           angular=SimpleNamespace(z=steer)))

    def latest(self, converter):
        return converter.pub.publish.call_args.args[0].drive

    def test_no_command_keeps_vehicle_stopped(self):
        converter = self.module.Converter()
        Stamp.now_value = 0.02
        converter.tick(None)
        self.assertEqual(self.latest(converter).speed, 0.0)

    def test_fresh_command_runs_then_stale_command_stops(self):
        converter = self.module.Converter()
        self.command(converter)
        Stamp.now_value = 0.02
        converter.tick(None)
        self.assertGreater(self.latest(converter).speed, 0.0)
        Stamp.now_value = 0.6
        converter.tick(None)
        self.assertEqual(self.latest(converter).speed, 0.0)
        self.command(converter)
        Stamp.now_value = 0.62
        converter.tick(None)
        self.assertAlmostEqual(self.latest(converter).speed, 0.05)

    def test_zero_and_invalid_commands_stop_before_timer(self):
        for value in (0.0, float('nan')):
            converter = self.module.Converter()
            converter.motion.speed = 2.4
            self.command(converter, speed=value)
            self.assertEqual(self.latest(converter).speed, 0.0)

    def test_simulation_clock_reset_requires_new_command(self):
        converter = self.module.Converter()
        Stamp.now_value = 1.0
        self.command(converter)
        converter.tick(None)
        Stamp.now_value = 0.1
        converter.tick(None)
        self.assertEqual(self.latest(converter).speed, 0.0)
        self.assertIsNone(converter.received)

    def test_baseline_passes_commands_directly(self):
        self.params['~race_profile'] = False
        converter = self.module.Converter()
        self.command(converter, speed=1.2, steer=0.15)
        self.assertEqual(self.latest(converter).speed, 1.2)
        self.assertEqual(self.latest(converter).steering_angle, 0.15)
        self.ros.Timer.assert_not_called()

    def test_watchdog_without_smoothing_preserves_command_and_stops_on_timeout(self):
        self.params.update({'~race_profile': False, '~command_watchdog': True})
        converter = self.module.Converter()
        self.command(converter, speed=2.8, steer=0.25)
        Stamp.now_value = 0.02
        converter.tick(None)
        self.assertEqual(self.latest(converter).speed, 2.8)
        self.assertEqual(self.latest(converter).steering_angle, 0.25)
        Stamp.now_value = 0.6
        converter.tick(None)
        self.assertEqual(self.latest(converter).speed, 0.0)

    def test_watchdog_preserves_zero_speed_steering(self):
        self.params.update({'~race_profile': False, '~command_watchdog': True})
        converter = self.module.Converter()
        self.command(converter, speed=0.0, steer=0.25)
        self.assertEqual(self.latest(converter).speed, 0.0)
        self.assertEqual(self.latest(converter).steering_angle, 0.25)

    def guard(self):
        self.params.update({'~race_profile': False, '~corner_speed_control': True, '~corner_speed': 1.6})
        with patch.dict(sys.modules, {'nav_msgs.msg': SimpleNamespace(Path=object, Odometry=object),
                                     'sensor_msgs.msg': SimpleNamespace(JointState=object)}):
            return self.module.Converter()

    def test_guard_requires_fresh_odometry(self):
        converter = self.guard()
        self.command(converter, speed=2.0, steer=0.1)
        Stamp.now_value = 0.02
        converter.tick(None)
        self.assertEqual(self.latest(converter).speed, 0.0)

    def test_guard_preserves_steering_and_zero_stop(self):
        converter = self.guard()
        for i in range(1, 100):
            Stamp.now_value = i*0.02
            converter.odom_received = converter.joint_received = converter.plan_received = Stamp.now()
            converter.joint_steer = 0.25
            converter.preview = [(0, 1.0), (3, 0)]
            self.command(converter, speed=2.0, steer=0.25)
            converter.tick(None)
        self.assertLessEqual(self.latest(converter).speed, 1.6)
        self.assertEqual(self.latest(converter).steering_angle, 0.25)
        self.command(converter, speed=0.0, steer=0.1)
        self.assertEqual(self.latest(converter).speed, 0.0)
        self.assertEqual(converter.governor.speed, 0.0)

    def test_guard_and_legacy_smoothing_cannot_stack(self):
        self.params['~corner_speed_control'] = True
        with self.assertRaises(ValueError): self.module.Converter()

    def test_shutdown_stops_and_shuts_timer(self):
        converter = self.module.Converter()
        converter.motion.speed = 2.4
        converter.stop()
        converter.timer.shutdown.assert_called_once()
        self.assertEqual(self.latest(converter).speed, 0.0)


if __name__ == '__main__':
    unittest.main()
