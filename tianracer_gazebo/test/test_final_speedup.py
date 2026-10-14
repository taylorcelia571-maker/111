import ast
import importlib.util
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from xml.etree import ElementTree as ET
import numpy as np

PACKAGE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PACKAGE/'scripts'))
from race_line import ClosedPath
from predictive_follower import PredictiveFollower
from controller_score import controller_speed_profile
from cycle_telemetry import CycleTelemetry
from analyze_final_race import estimate_braking,summarize_cycles,estimate_steering_response,read_bag,TOPICS


def follower(**kwargs):
    theta=np.linspace(0,2*math.pi,1000)
    path=ClosedPath(np.column_stack((6*np.cos(theta),6*np.sin(theta))),spacing=.04)
    return PredictiveFollower(path,max_speed=10,lateral_accel=9.94,acceleration=7.3,
        braking=3.7,yaw_rate=4.4,response_delay=.25,speed_planning='delayed_braking',
        lateral_control='race_feedback',steering_feedback_gain=.25,
        fast_start=True,start_blend=True,startup_release_early=True,**kwargs)


class ConnectorTests(unittest.TestCase):
    def test_default_and_invalid_length(self):
        self.assertEqual(follower().connector_length,2.5)
        for length in (1.9,3.1,math.nan,math.inf):
            with self.assertRaises(ValueError):follower(startup_connector_length=length)

    def test_fractional_endpoint_does_not_trap_projection(self):
        f=follower(startup_connector_length=2.4)
        f.origin=27.5678;f.progress=f.origin+2.4;f.connector_offset=np.array([.07,0.])
        self.assertFalse(f.connector_active())
        f.progress-=.01
        self.assertTrue(f.connector_active())

    def test_quintic_endpoint_position_first_second_derivatives(self):
        f=follower(startup_connector_length=2.4);f.origin=0.;f.connector_offset=np.array([.07,.01])
        def offset(s):return f.sample_tracking(s)-f.path.sample(s)
        h=1e-4
        np.testing.assert_allclose(offset(0),f.connector_offset,atol=1e-12)
        np.testing.assert_allclose(offset(f.connector_length),[0,0],atol=1e-12)
        for s in (0.,f.connector_length):
            self.assertLess(np.linalg.norm((offset(s+h)-offset(s-h))/(2*h)),1e-6)
            self.assertLess(np.linalg.norm((offset(s+h)-2*offset(s)+offset(s-h))/h**2),1e-4)

    def test_start_resets_readiness_and_connector_state(self):
        f=follower(startup_connector_length=2.4)
        f.launch_ready=2.;f.alignment_ready=3.;f.launch_limited=True
        f.start(6.07,0,math.pi/2)
        self.assertEqual(f.launch_ready,0);self.assertEqual(f.alignment_ready,0)
        self.assertTrue(f.launch_limited)

    def test_initial_speed_cap_and_acceleration_are_not_skipped(self):
        f=follower(startup_speed=2.,startup_acceleration=2.5)
        f.start(6.07,0,math.pi/2);f.measured_steer=0.;f.measured_steer_age=0.
        v,_=f.step(6.07,0,math.pi/2,0.,.02)
        self.assertLessEqual(v,2.5*.02+1e-12)
        self.assertLessEqual(v,2.)
        self.assertTrue(f.launch_limited)


class ScoreTests(unittest.TestCase):
    def test_proxy_keeps_speed_caps_and_response_budget(self):
        f=follower(steering_preview_time=.12)
        before=f.path.speeds.copy();profile=controller_speed_profile(f)
        self.assertTrue(np.isfinite(profile).all());self.assertTrue((profile>0).all())
        self.assertTrue((profile<=before+1e-12).all())
        np.testing.assert_array_equal(f.path.speeds,before)
        self.assertEqual(f.response_delay,.25)


class TelemetryTests(unittest.TestCase):
    def test_flush_preserves_events_and_nonfinite_values_are_explicit_null(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'cycles.jsonl';writer=CycleTelemetry(str(path))
            self.assertTrue(writer.append({'event':'cycle','value':math.nan}))
            self.assertTrue(writer.flush())
            self.assertTrue(writer.close())
            self.assertIsNone(json.loads(path.read_text())['value'])
            self.assertEqual(writer.status()['dropped'],0)
            with self.assertRaises(FileExistsError):CycleTelemetry(str(path))

    def test_saturation_never_blocks_control_append(self):
        with tempfile.TemporaryDirectory() as directory:
            writer=CycleTelemetry(str(Path(directory)/'cycles.jsonl'),capacity=1)
            for i in range(2000):writer.append({'index':i})
            writer.flush();writer.close()
            self.assertEqual(writer.accepted+writer.dropped,2000)


class AnalysisTests(unittest.TestCase):
    def test_command_drop_is_not_reported_as_measured_braking(self):
        row={'event':'cycle','simulation_time':1.,'elapsed':0.,'progress':0.,
             'measured_speed':3.,'output_speed':3.,'control':{'reason':'planned_envelope'}}
        other=dict(row,simulation_time=1.02,elapsed=.02,output_speed=1.)
        r=summarize_cycles([row,other])
        self.assertEqual(r['maximum_single_sample_command_drop'],2.)
        self.assertTrue(r['command_drop_rate_is_not_measured_braking'])
        self.assertAlmostEqual(sum(r['reason_seconds'].values()),.02)

    def test_measured_fit_is_not_permission_to_raise_budget(self):
        commands=[{'t':0.,'v':5.},{'t':.1,'v':2.}]
        odom=[{'t':t,'v':5.-3.7*max(0,t-.35),'omega':0.,'lateral':0.}
              for t in np.arange(0,1.2,.05)]
        r=estimate_braking(commands,odom)
        self.assertTrue(r['available'])
        self.assertAlmostEqual(r['minimum_observed_deceleration'],3.7,places=5)
        self.assertFalse(r['permission_to_raise_braking'])
        for v in odom:v['omega']=.8
        self.assertFalse(estimate_braking(commands,odom)['available'])

    def test_nonfinite_odometry_rejects_braking_evidence(self):
        r=estimate_braking([{'t':0.,'v':5.},{'t':1.,'v':2.}],
                          [{'t':1.,'v':math.nan,'omega':0.,'lateral':0.}])
        self.assertFalse(r['available']);self.assertFalse(r['permission_to_raise_braking'])

    def test_gradual_commanded_braking_is_not_missed_by_single_step_filter(self):
        commands=[dict(t=float(t),v=5.-3.7*t) for t in np.arange(0,.8,.02)]
        odom=[dict(t=float(t),v=5.-3.7*t,omega=0.,lateral=0.) for t in np.arange(0,1.3,.05)]
        r=estimate_braking(commands,odom)
        self.assertTrue(r['available'])
        self.assertAlmostEqual(r['minimum_observed_deceleration'],3.7,places=5)
        self.assertFalse(r['permission_to_raise_braking'])

    def test_continuous_measured_steering_fit_is_effective_not_certified(self):
        times=np.arange(0,8,.02)
        commands=[dict(t=float(t),steer=.2*math.sin(t)+.1*math.sin(3*t)) for t in times]
        actual=0.;joints=[]
        for t in times:
            joints.append(dict(t=float(t),angle=actual))
            delayed=.2*math.sin(max(0,t-.02))+.1*math.sin(3*max(0,t-.02))
            actual+=(delayed-actual)*.02/.15
        r=estimate_steering_response(commands,joints)
        self.assertTrue(r['available'])
        self.assertAlmostEqual(r['fit']['effective_time_constant_seconds'],.15,delta=.03)
        self.assertFalse(r['permission_to_change_safety'])

    def test_steering_clock_reset_is_rejected(self):
        commands=[dict(t=float(i),steer=.2) for i in range(60)]
        commands[-1]['t']=-1.
        joints=[dict(t=float(i),angle=.1) for i in range(60)]
        r=estimate_steering_response(commands,joints)
        self.assertFalse(r['available'])
        self.assertIn('Clock reset',r['reason'])


class BoundaryTests(unittest.TestCase):
    def test_protected_functions_match_71_base(self):
        hashes={
            'continuous_race.py':{'sensors':'d9ed232bd9703c400bd176ae50e581fe89778d2feeb0806f027a19e41189ab78',
                'verified_laser_history':'7ca662197773f1dd16f85104c6b53989de973a9a0de95c81966b3587ca206316',
                'laser_feedback':'9c63002dd9f722d6b3febaa3b1a0c4e004ad72f6d9ce63357b5a1fa5609fd85a'},
            'loop_planner.py':{'check_vehicle_clearance':'d075be38e71de38b8e91ab9617647516ab07add21f2b5bba1e0c6c7850ac9954',
                'segment_check':'106cd493a0928cdafb093de6a5afb5d4ecaf01d6b345696161fa08a1022232c5',
                'feasible':'99b219e5e2c38f92b8c97d46f2725b18ad55a796af86318061dc6eea7b907982'},
            'predictive_follower.py':{'reference_clearance':'c50e1f3bea90ae0aeed5bd575bdbd23990bc080f6cfa36afb1dd8de461351997'}}
        for file,wanted in hashes.items():
            source=(PACKAGE/'scripts'/file).read_text()
            found={n.name:hashlib.sha256(ast.get_source_segment(source,n).encode()).hexdigest()
                   for n in ast.walk(ast.parse(source)) if isinstance(n,ast.FunctionDef) and n.name in wanted}
            self.assertEqual(found,wanted)

    def test_frozen_launch_budgets_and_feature_rollback(self):
        root=ET.parse(PACKAGE/'launch/final_speedup.launch').getroot()
        args={a.attrib['name']:a.attrib['default'] for a in root.findall('arg')}
        self.assertEqual(args['lateral_accel'],'9.94');self.assertEqual(args['braking'],'3.7')
        self.assertEqual(args['startup_connector_length'],'2.5');self.assertEqual(args['version'],'baseline')
        includes={a.attrib['name']:a.attrib['value'] for a in root.find('include').findall('arg')}
        for k,v in {'route_margin':'0.34','response_delay':'0.25','steering_rate':'2.0'}.items():self.assertEqual(includes[k],v)
        self.assertIn("arg('legacy_control')",includes['route_control_scoring'])
        self.assertIn("arg('legacy_control')",includes['startup_connector_length'])


@unittest.skipUnless(importlib.util.find_spec('rosbag'), 'Native Noetic rosbag required')
class NativeBagTests(unittest.TestCase):
    def test_real_bag_roundtrip_preserves_commands_and_feedback_without_control(self):
        import rosbag
        import rospy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import JointState
        self.assertNotIn('/gazebo/model_states',TOPICS)
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'synthetic_test_only.bag'
            with rosbag.Bag(str(path),'w') as bag:
                command=Twist();command.linear.x=2.;command.angular.z=.1
                bag.write('/tianracer/cmd_vel',command,rospy.Time.from_sec(1.))
                odom=Odometry();odom.header.stamp=rospy.Time.from_sec(1.05)
                odom.twist.twist.linear.x=2.1;odom.pose.pose.position.x=.1
                bag.write('/tianracer/odom',odom,odom.header.stamp)
                joints=JointState();joints.header.stamp=rospy.Time.from_sec(1.05)
                joints.name=['left_steering_hinge_joint','right_steering_hinge_joint']
                joints.position=[.1,.1]
                bag.write('/tianracer/joint_states',joints,joints.header.stamp)
            result=read_bag(path)
            self.assertEqual(result['commands'][0]['v'],2.)
            self.assertEqual(result['topic_counts']['/tianracer/odom'],1)
            self.assertEqual(result['observations'][0]['v'],2.1)
            self.assertFalse(result['braking']['permission_to_raise_braking'])


if __name__=='__main__':unittest.main()
