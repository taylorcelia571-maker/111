"""A timer must never publish a world-frame Twist as body-frame odometry."""
import copy
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock,patch
import numpy as np


def vector(): return NS(x=0.,y=0.,z=0.)
def pose(): return NS(position=vector(),orientation=NS(x=0.,y=0.,z=0.,w=1.))
def twist(): return NS(linear=vector(),angular=vector())
def transform(**kw):
    data=dict(translation=vector(),rotation=NS(x=0.,y=0.,z=0.,w=1.));data.update(kw)
    return NS(**data)
def stamped(**kw):
    data=dict(transform=transform(),header=NS());data.update(kw)
    return NS(**data)
def odometry(): return NS(header=NS(),pose=NS(pose=pose()),twist=NS(twist=twist()))


class SnapshotTests(unittest.TestCase):
    def setUp(self):
        self.now=1.;self.hook=None;self.messages=[]
        ros=Mock();ros.get_param.return_value='tianracer'
        ros.Time.now.side_effect=lambda:self.now
        ros.Publisher.return_value.publish.side_effect=lambda m:self.messages.append(copy.deepcopy(m))
        inverse=lambda q:np.array([-q[0],-q[1],-q[2],q[3]])/np.dot(q,q)
        modules={'rospy':ros,'nav_msgs.msg':NS(Odometry=odometry),
                 'geometry_msgs.msg':NS(Pose=pose,Twist=twist,Transform=transform,TransformStamped=stamped,
                                        Vector3Stamped=lambda:NS(vector=vector())),
                 'gazebo_msgs.msg':NS(LinkStates=object),'std_msgs.msg':NS(Header=lambda **kw:NS(**kw)),
                 'tf':NS(transformations=NS(quaternion_inverse=inverse)),
                 'tf2_ros':NS(TransformBroadcaster=Mock),
                 'tf2_geometry_msgs':NS(do_transform_vector3=self.convert)}
        with patch.dict(sys.modules,modules):
            spec=importlib.util.spec_from_file_location('odometry_snapshot_test',Path(__file__).resolve().parents[1]/'scripts/gazebo_odometry.py')
            mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
        self.node=mod.OdometryNode()

    def convert(self,msg,tf):
        if self.hook is not None:
            hook=self.hook;self.hook=None;hook()
        q=tf.transform.rotation;x,y,z,w=q.x,q.y,q.z,q.w
        r=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                    [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                    [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
        v=r.dot([msg.vector.x,msg.vector.y,msg.vector.z])
        return NS(vector=NS(x=v[0],y=v[1],z=v[2]))

    def sample(self,world_speed=-2.,position=2.):
        p=pose();p.position.x=position;p.orientation.z=1.;p.orientation.w=0.
        t=twist();t.linear.x=world_speed
        return NS(name=['tianracer::tianracer/base_footprint'],pose=[p],twist=[t])

    def test_timer_interleaving_retains_prior_complete_body_snapshot(self):
        self.node.sub_robot_pose_update(self.sample(-1.,1.))
        self.now=2.;self.hook=lambda:self.node.timer_callback(None)
        self.node.sub_robot_pose_update(self.sample(-2.,2.))
        prior=self.messages[-1]
        self.assertEqual(prior.header.stamp,1.)
        self.assertEqual(prior.pose.pose.position.x,1.)
        self.assertAlmostEqual(prior.twist.twist.linear.x,1.)
        self.node.timer_callback(None);current=self.messages[-1]
        self.assertEqual(current.header.stamp,2.)
        self.assertEqual(current.pose.pose.position.x,2.)
        self.assertAlmostEqual(current.twist.twist.linear.x,2.)

    def test_real_reverse_is_preserved_and_input_message_not_mutated(self):
        data=self.sample(2.)
        self.node.sub_robot_pose_update(data);self.node.timer_callback(None)
        self.assertAlmostEqual(self.messages[-1].twist.twist.linear.x,-2.)
        self.assertEqual(data.twist[0].linear.x,2.)

    def test_angular_velocity_uses_the_same_body_frame(self):
        data=self.sample();data.twist[0].angular.x=3.
        self.node.sub_robot_pose_update(data);self.node.timer_callback(None)
        self.assertAlmostEqual(self.messages[-1].twist.twist.angular.x,-3.)

    def test_absent_vehicle_never_creates_identity_odometry(self):
        data=self.sample();data.name=['ground_plane::link']
        self.node.sub_robot_pose_update(data);self.node.timer_callback(None)
        self.assertEqual(self.messages,[])


if __name__=='__main__':unittest.main()
