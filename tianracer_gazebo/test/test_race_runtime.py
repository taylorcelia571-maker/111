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

    def test_generated_scan_transform_uses_acquisition_timestamp(self):
        old = self.node
        self.node = self.module.ContinuousRace(old.path)
        self.node.on_scan(old.scan)
        self.node.scan.header.stamp = Stamp(0.9)
        self.node.on_odom(old.odom)
        listener = self.node.listener
        listener.lookupTransform.side_effect = lambda target,source,stamp: (
            ([3.1,0,0],[0,0,math.pi/2+0.1,1]) if source == 'laser'
            else ([3,0,0],[0,0,math.pi/2,1]))
        self.assertTrue(self.node.start(None).success)
        with patch('predictive_follower.reference_clearance',return_value=float('inf')) as guard:
            self.tick()
            relative = guard.call_args.args[4]
        self.assertAlmostEqual(relative[0],0.13)
        self.assertAlmostEqual(relative[1],-0.1)
        self.assertAlmostEqual(relative[2],0.1)
        listener.lookupTransform.assert_any_call('map','laser',self.node.scan.header.stamp)

    def generated_node(self):
        old = self.node
        self.node = self.module.ContinuousRace(old.path)
        self.node.on_scan(old.scan)
        self.node.on_odom(old.odom)
        self.assertTrue(self.node.start(None).success)

    def refresh_at(self, seconds):
        Stamp.now_value = seconds
        self.node.scan.header.stamp = Stamp(seconds)
        self.node.odom.header.stamp = Stamp(seconds)
        self.node.on_scan(self.node.scan)
        self.node.on_odom(self.node.odom)
        self.node.listener.getLatestCommonTime.return_value = Stamp(seconds)

    def test_duplicate_clock_skips_tick_then_resumes(self):
        self.generated_node()
        self.node.previous = Stamp(1)
        progress = self.node.follower.progress
        self.tick()
        self.assertTrue(self.node.active)
        self.node.pub.publish.assert_not_called()
        self.assertEqual(self.node.follower.progress, progress)
        self.refresh_at(1.02)
        self.tick()
        self.assertTrue(self.node.active)
        self.assertGreater(self.node.pub.publish.call_args.args[0].linear.x, 0)

    def test_fresh_delayed_cycle_projects_actual_arc_without_acceleration_catchup(self):
        self.generated_node()
        self.node.previous = Stamp(1)
        self.refresh_at(1.25)
        theta = .15
        self.node.listener.lookupTransform.side_effect = lambda target,source,stamp: (
            ([3*math.cos(theta),3*math.sin(theta),0],[0,0,math.pi/2+theta,1]))
        with patch.object(self.node.follower,'step',wraps=self.node.follower.step) as step:
            self.tick()
        self.assertTrue(self.node.active)
        self.assertAlmostEqual(step.call_args.args[-1],.04)
        self.assertAlmostEqual(step.call_args.kwargs['projection_dt'],.25)
        self.assertLessEqual(self.node.pub.publish.call_args.args[0].linear.x,.08)
        self.assertGreater(self.node.follower.progress-self.node.follower.origin,.3)

    def test_excessive_gap_latches_stop_even_with_fresh_feedback(self):
        self.generated_node()
        self.node.previous = Stamp(1)
        self.refresh_at(1.4)
        self.tick()
        self.assertFalse(self.node.active)
        self.assertEqual(self.node.pub.publish.call_args.args[0].linear.x,0)
        self.assertIn('dt=0.400000',self.ros.logerr.call_args.args[1].args[0])

    def test_clock_rewind_latches_stop(self):
        self.generated_node()
        self.node.previous = Stamp(1.1)
        self.tick()
        self.assertFalse(self.node.active)
        self.assertEqual(self.node.pub.publish.call_args.args[0].linear.x,0)

    def test_nonfinite_feedback_latches_stop(self):
        self.generated_node()
        self.node.odom.twist.twist.linear.x = float('nan')
        self.tick()
        self.assertFalse(self.node.active)
        self.assertEqual(self.node.pub.publish.call_args.args[0].linear.x,0)

    def test_status_reports_stale_topic_without_starting_or_disappearing(self):
        self.node.scan.header.stamp = Stamp(.5)
        response=self.node.status(None)
        self.assertFalse(response.success)
        self.assertIn('scan timing invalid',response.message)
        self.assertIn('stamp_age=0.500000',response.message)
        self.assertFalse(self.node.start(None).success)
        self.assertFalse(self.node.active)
        self.node.scan.header.stamp=Stamp(1)
        self.assertTrue(self.node.status(None).success)
        self.assertTrue(self.node.start(None).success)

    def test_small_future_transport_skew_allowed_but_large_skew_rejected(self):
        self.node.scan.header.stamp=Stamp(1.01)
        self.assertTrue(self.node.status(None).success)
        self.node.scan.header.stamp=Stamp(1.05)
        self.assertFalse(self.node.status(None).success)
        self.assertIn('scan timing invalid',self.node.status(None).message)

    def test_status_identifies_odom_and_uninitialized_clock(self):
        self.node.odom.header.stamp=Stamp(.4)
        self.assertIn('odom timing invalid',self.node.status(None).message)
        Stamp.now_value=0
        self.assertIn('Waiting for initialized ROS clock',self.node.status(None).message)

    def test_generated_budget_parameters_reach_controller_and_status(self):
        self.params.update({'~max_speed':5.0,'~lateral_accel':6.0,'~acceleration':3.0,
                            '~yaw_rate':4.0,'~steering_rate':2.5})
        self.generated_node()
        report=self.node.follower.plan_report()
        self.assertEqual(report['limits']['yaw_rate'],4.0)
        self.assertEqual(report['limits']['steering_rate'],2.5)
        self.tick()
        self.assertIn('measured_speed',self.node.status(None).message)
        self.assertIn('planned_speed',self.node.status(None).message)

    def test_start_connector_map_rejection_falls_back_before_motion(self):
        self.params['~start_blend']=True
        old=self.node
        self.node=self.module.ContinuousRace(old.path,start_path_guard=lambda f:False)
        self.node.on_scan(old.scan);self.node.on_odom(old.odom)
        original=self.node.follower.start
        def connector_start(x,y,yaw):
            original(x,y,yaw)
            self.node.follower.connector_offset=np.array([.01,0])
            self.node.follower.departure=False
        with patch.object(self.node.follower,'start',side_effect=connector_start):
            self.assertTrue(self.node.start(None).success)
        self.assertIsNone(self.node.follower.connector_offset)
        self.assertTrue(self.node.follower.departure)
        self.node.pub.publish.assert_not_called()

    def test_pose_prediction_is_short_and_timestamp_matched(self):
        self.params['~predict_pose']=True
        old=self.node
        self.node=self.module.ContinuousRace(old.path)
        self.node.on_scan(old.scan);self.node.on_odom(old.odom)
        self.node.odom.header.stamp=Stamp(.95)
        self.node.odom.twist.twist.linear.x=2
        self.node.odom.twist.twist.angular=NS(z=0)
        self.node.listener.getLatestCommonTime.return_value=Stamp(.95)
        pose=self.node.sensors()
        self.assertAlmostEqual(pose[0],3)
        self.assertAlmostEqual(pose[1],-.13+.1)
        self.assertTrue(self.node.feedback_pose['prediction_applied'])
        self.assertAlmostEqual(self.node.feedback_pose['raw_rear_pose'][1],-.13)
        self.node.odom.header.stamp=Stamp(.85)
        self.node.listener.getLatestCommonTime.return_value=Stamp(.85)
        self.assertAlmostEqual(self.node.sensors()[1],-.13)
        self.node.listener.getLatestCommonTime.return_value=Stamp(.95)
        self.assertAlmostEqual(self.node.sensors()[1],-.13)  # stamp mismatch: no extrapolation

    def test_pose_prediction_rejects_invalid_yaw_rate(self):
        self.params['~predict_pose']=True
        old=self.node;self.node=self.module.ContinuousRace(old.path)
        self.node.on_scan(old.scan);self.node.on_odom(old.odom)
        self.node.odom.twist.twist.angular=NS(z=float('nan'))
        self.assertFalse(self.node.start(None).success)

    def test_verified_laser_cache_avoids_blocking_then_stops_if_tf_reacquisition_fails(self):
        self.generated_node()
        self.tick()
        self.assertIsNotNone(self.node.laser_cache)
        self.node.listener.canTransform.return_value=False
        self.node.listener.waitForTransform.reset_mock()
        import copy
        self.node.scan=copy.deepcopy(self.node.scan)
        self.refresh_at(1.04)
        self.tick()
        self.assertTrue(self.node.active)
        self.node.listener.waitForTransform.assert_not_called()
        self.refresh_at(1.15)
        self.node.listener.waitForTransform.side_effect=RuntimeError('TF unavailable')
        self.tick()
        self.assertFalse(self.node.active)
        self.node.listener.waitForTransform.assert_called_once()
        self.assertEqual(self.node.pub.publish.call_args.args[0].linear.x,0)

    def test_fresh_scan_reacquired_when_short_cache_expires(self):
        import copy
        self.generated_node();self.tick()
        self.node.scan=copy.deepcopy(self.node.scan)
        self.refresh_at(1.15)
        self.node.listener.canTransform.return_value=False
        self.tick()
        self.assertTrue(self.node.active)
        self.node.listener.waitForTransform.assert_called_once()
        self.assertIs(self.node.laser_cache[0],self.node.scan)
        self.assertGreater(self.node.pub.publish.call_args.args[0].linear.x,0)

    def test_recent_transformable_history_selected_without_waiting_for_latest_scan(self):
        import copy
        self.generated_node()
        recent=copy.deepcopy(self.node.scan)
        recent.header.stamp=Stamp(.97)
        latest=copy.deepcopy(self.node.scan)
        self.node.on_scan(recent);self.node.on_scan(latest)
        self.node.listener.canTransform.side_effect=lambda target,source,stamp:stamp.to_sec()==.97
        self.tick()
        self.assertTrue(self.node.active)
        self.assertIs(self.node.laser_cache[0],recent)
        self.node.listener.waitForTransform.assert_not_called()

    def test_stale_history_and_current_scan_cannot_restart_motion(self):
        self.generated_node();self.tick()
        Stamp.now_value=1.5
        self.tick()
        self.assertFalse(self.node.active)
        self.assertEqual(self.node.pub.publish.call_args.args[0].linear.x,0)

    def test_tf_wait_refreshes_control_time_before_integrating(self):
        self.generated_node()
        self.node.listener.canTransform.return_value=False
        self.node.listener.waitForTransform.side_effect=lambda *args:setattr(Stamp,'now_value',1.03)
        with patch.object(self.node.follower,'step',wraps=self.node.follower.step) as step:
            self.tick()
        self.assertTrue(self.node.active)
        self.assertAlmostEqual(self.node.previous.to_sec(),1.03)
        self.assertAlmostEqual(step.call_args.args[-1],.04)
        self.assertAlmostEqual(step.call_args.kwargs['projection_dt'],.05)


if __name__ == '__main__': unittest.main()
