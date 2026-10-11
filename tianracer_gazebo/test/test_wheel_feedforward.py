"""Rigid-body rolling constraints and ROS publisher lifecycle regression."""
import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as E
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from wheel_kinematics import wheel_commands, equivalent_steer


class WheelTests(unittest.TestCase):
    def test_straight_all_wheels_equal(self):
        c = wheel_commands(2, 0, True)
        for key in ('lr','rr','lf','rf'): self.assertAlmostEqual(c[key], 2/.032)
        self.assertEqual(c['left'], 0)

    def test_inner_outer_wheel_order_and_mirror_symmetry(self):
        a,b = wheel_commands(2,.35,True), wheel_commands(2,-.35,True)
        self.assertLess(a['lr'],a['rr'])
        self.assertLess(a['lf'],a['rf'])
        for left,right in [('lr','rr'),('lf','rf')]: self.assertAlmostEqual(a[left],b[right])
        self.assertAlmostEqual(a['left'],-b['right'])

    def test_reverse_speed_sign_and_zero_no_drive(self):
        a,b = wheel_commands(2,.4,True), wheel_commands(-2,.4,True)
        for key in ('lr','rr','lf','rf'): self.assertAlmostEqual(a[key],-b[key])
        z = wheel_commands(0,.4,True)
        for key in ('lr','rr','lf','rf'): self.assertEqual(z[key],0)

    def test_front_rolling_velocity_matches_rigid_body(self):
        v,delta=2,.35
        c = wheel_commands(v,delta,True)
        omega = v*math.tan(delta)/.26
        for key,side,angle in [('lf',1,c['left']),('rf',-1,c['right'])]:
            x = .26-side*.025*math.sin(angle)
            y = side*.065+side*.025*math.cos(angle)
            vx,vy = v-omega*y, omega*x
            self.assertAlmostEqual(c[key]*.032,vx*math.cos(angle)+vy*math.sin(angle))
            self.assertAlmostEqual(-vx*math.sin(angle)+vy*math.cos(angle),0,places=8)

    def test_model_hinge_limits_and_inverse_steering(self):
        for delta in (-.5,-.1,0,.2,.5):
            c=wheel_commands(2,delta,True)
            self.assertLess(abs(c['left']),.6)
            self.assertLess(abs(c['right']),.6)
            self.assertAlmostEqual(equivalent_steer(c['left'],c['right']),delta)

    def test_baseline_is_unchanged_and_invalid_stops(self):
        c=wheel_commands(2,.3,False)
        for key in ('lr','rr','lf','rf'): self.assertEqual(c[key],2/.032)
        expected=math.atan2(.265*math.tan(.3),.265-.13*math.tan(.3)/2)
        self.assertAlmostEqual(c['left'],expected)
        self.assertTrue(all(v==0 for v in wheel_commands(float('nan'),0,True).values()))

    def test_geometry_matches_unchanged_urdf(self):
        root=E.parse(Path(__file__).resolve().parents[1]/'urdf/tianracer.xacro').getroot()
        joints={j.attrib['name']:j for j in root.findall('joint')}
        left=joints['left_steering_hinge_joint'].find('origin').attrib['xyz'].split()
        rear=joints['left_rear_wheel_joint'].find('origin').attrib['xyz'].split()
        self.assertAlmostEqual(float(left[0])-float(rear[0]),.26)
        self.assertAlmostEqual(float(left[1]),.065)
        self.assertAlmostEqual(float(rear[1]),.09)


class ServoTests(unittest.TestCase):
    def setUp(self):
        self.ros=Mock()
        self.ros.get_param.return_value=True
        self.ros.Publisher.side_effect=lambda *a,**k:Mock()
        modules={'rospy':self.ros,'std_msgs.msg':NS(Float64=object),
                 'std_srvs.srv':NS(Empty=object,EmptyResponse=lambda:NS()),
                 'ackermann_msgs.msg':NS(AckermannDriveStamped=object)}
        with patch.dict(sys.modules,modules):
            spec=importlib.util.spec_from_file_location('servo_test',Path(__file__).resolve().parents[1]/'scripts/servo_commands.py')
            self.module=importlib.util.module_from_spec(spec);spec.loader.exec_module(self.module)
        self.servo=self.module.ServoCommands()

    def test_publishers_created_once_and_brake_latches(self):
        msg=NS(drive=NS(speed=2,steering_angle=.3))
        for _ in range(20):self.servo.command(msg)
        self.assertEqual(self.ros.Publisher.call_count,6)
        self.servo.emergency_brake(None)
        self.servo.command(msg)
        for p in self.servo.publishers.values():self.assertEqual(p.publish.call_args.args[0],0)
        self.servo.emergency_brake(None)
        self.servo.command(msg)
        self.assertGreater(self.servo.publishers['lr'].publish.call_args.args[0],0)

    def test_shutdown_publishes_zero_and_remains_braking(self):
        self.servo.stop()
        self.assertTrue(self.servo.braking)
        for p in self.servo.publishers.values():self.assertEqual(p.publish.call_args.args[0],0)


if __name__=='__main__':unittest.main()
