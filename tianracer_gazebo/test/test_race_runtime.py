"""ROS adapter tests with fake transport; not integration against ROS/Gazebo."""
import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
import numpy as np
from test_race_converter import Stamp

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))


def twist(): return NS(linear=NS(x=0.0), angular=NS(z=0.0))


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        Stamp.now_value = 1.0
        self.params = {'~path_file': '/fake/path.csv'}
        self.ros = Mock()
        self.ros.Time = Stamp
        self.ros.Duration = Stamp
        self.ros.get_param.side_effect = lambda key, default=None: self.params.get(key, default)
        self.ros.get_namespace.return_value = '/tianracer/'
        self.ros.get_name.return_value = '/tianracer/continuous_race'
        self.ros.resolve_name.return_value = '/tianracer/cmd_vel'
        self.graph = Mock()
        self.graph.MasterError = self.graph.MasterFailure = RuntimeError
        self.graph.Master.return_value.getSystemState.return_value = ([], [], [])
        self.ros.exceptions.ROSTimeMovedBackwardsException = RuntimeError
        self.transform = Mock()
        self.transform.Exception = RuntimeError
        listener = self.transform.TransformListener.return_value
        listener.getLatestCommonTime.return_value = Stamp(1.0)
        listener.lookupTransform.side_effect = lambda target, source, stamp: (
            ([3.0, 0, 0], [0, 0, math.pi/2, 1]) if target == 'map'
            else ([0.1, 0, 0], [0, 0, 0, 1]))
        self.transform.transformations.euler_from_quaternion.side_effect = lambda q: (0, 0, q[2])
        modules = {'rospy': self.ros, 'rosgraph': self.graph, 'tf': self.transform,
                   'geometry_msgs.msg': NS(Twist=twist), 'nav_msgs.msg': NS(Odometry=object),
                   'sensor_msgs.msg': NS(LaserScan=object),
                   'std_srvs.srv': NS(Trigger=object, TriggerResponse=lambda s,m:NS(success=s,message=m))}
        with patch.dict(sys.modules, modules):
            spec = importlib.util.spec_from_file_location('continuous_runtime_test', SCRIPTS/'continuous_race.py')
            self.module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(self.module)
        t = np.linspace(0, 2*math.pi, 500)
        with patch.object(self.module.np, 'loadtxt', return_value=np.column_stack([3*np.cos(t),3*np.sin(t)])):
            self.node = self.module.ContinuousRace()
        self.node.on_scan(NS(header=NS(stamp=Stamp(1),frame_id='laser'),ranges=np.full(181,np.inf),
                             angle_min=-math.pi/2,angle_increment=math.pi/180,range_min=0.05,range_max=10))
        self.node.on_odom(NS(header=NS(stamp=Stamp(1)),twist=NS(twist=NS(linear=NS(x=0.0)))))

    def test_idle_until_explicit_start_and_double_start_rejected(self):
        self.assertFalse(self.node.active)
        self.node.pub.publish.assert_not_called()
        self.assertTrue(self.node.start(None).success)
        self.assertFalse(self.node.start(None).success)

    def test_missing_stale_and_moving_inputs_block_start(self):
        self.node.scan_time = None
        self.assertFalse(self.node.start(None).success)
        self.node.scan_time = Stamp(0.1)
        self.assertFalse(self.node.start(None).success)
        self.node.scan_time = Stamp(1)
        self.node.odom.twist.twist.linear.x = 1
        self.assertFalse(self.node.start(None).success)

    def tick(self):
        self.ros.is_shutdown.side_effect = [False, True]
        self.node.run()

    def test_first_cycle_publishes_valid_motion(self):
        # Start subtracts a duration; fake Stamp implements the needed API.
        self.assertTrue(self.node.start(None).success)
        self.tick()
        message = self.node.pub.publish.call_args.args[0]
        self.assertGreater(message.linear.x, 0)
        self.assertTrue(math.isfinite(message.angular.z))

    def test_invalid_scan_latches_stop(self):
        self.assertTrue(self.node.start(None).success)
        self.node.scan.ranges = np.zeros(181)
        self.tick()
        self.assertFalse(self.node.active)
        self.assertEqual(self.node.pub.publish.call_args.args[0].linear.x, 0)

    def test_sensor_dropout_latches_stop(self):
        self.assertTrue(self.node.start(None).success)
        Stamp.now_value = 1.5
        self.tick()
        self.assertFalse(self.node.active)
        self.assertEqual(self.node.pub.publish.call_args.args[0].linear.x, 0)

    def test_competing_controller_blocks_start(self):
        self.graph.Master.return_value.getSystemState.return_value = (
            [('/tianracer/cmd_vel', ['/tianracer/move_base'])], [], [])
        self.assertFalse(self.node.start(None).success)
        self.assertFalse(self.node.active)

    def test_explicit_stop_publishes_zero(self):
        self.assertTrue(self.node.start(None).success)
        self.assertTrue(self.node.stop(None).success)
        self.assertFalse(self.node.active)
        self.assertEqual(self.node.pub.publish.call_args.args[0].linear.x, 0)

    def test_generated_path_uses_rear_axle_and_waits_for_start(self):
        old = self.node
        self.node = self.module.ContinuousRace(old.path)
        self.node.on_scan(old.scan)
        self.node.on_odom(old.odom)
        self.assertAlmostEqual(self.node.sensors()[1], -0.13)
        self.assertFalse(self.node.active)
        self.assertTrue(self.node.start(None).success)
        self.tick()
        self.assertGreater(self.node.pub.publish.call_args.args[0].linear.x, 0)

    def test_generated_path_static_clearance_guard_latches_stop(self):
        old = self.node
        def reject(pose): raise ValueError('near wall')
        self.node = self.module.ContinuousRace(old.path, clearance_guard=reject)
        self.node.on_scan(old.scan)
        self.node.on_odom(old.odom)
        self.assertTrue(self.node.start(None).success)
        self.tick()
        self.assertFalse(self.node.active)
        self.assertEqual(self.node.pub.publish.call_args.args[0].linear.x, 0)


if __name__ == '__main__': unittest.main()
