"""Offline transport-adapter tests using fake messages/time, not a ROS runtime."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))


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

    def test_shutdown_stops_and_shuts_timer(self):
        converter = self.module.Converter()
        converter.motion.speed = 2.4
        converter.stop()
        converter.timer.shutdown.assert_called_once()
        self.assertEqual(self.latest(converter).speed, 0.0)


if __name__ == '__main__':
    unittest.main()
