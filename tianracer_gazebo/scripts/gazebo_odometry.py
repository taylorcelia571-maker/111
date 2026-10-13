#!/usr/bin/env python3

'''
This script makes Gazebo less fail by translating gazebo status messages to odometry data.
Since Gazebo also publishes data faster than normal odom data, this script caps the update to 20hz.
Winter Guerra
'''

import rospy
import copy
import threading
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped, Vector3Stamped
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Header
import numpy as np
import math
import tf
import tf2_ros
import tf2_geometry_msgs

robot_name = rospy.get_param('robot_name', default="tianracer")
class OdometryNode:
    # Set publishers
    pub_odom = rospy.Publisher('odom', Odometry, queue_size=1)

    def __init__(self):
        self.lock = threading.RLock()
        # init internals
        self.last_received_pose = Pose()
        self.last_received_pose.orientation.w = 1
        self.last_received_twist = Twist()
        self.last_recieved_stamp = None

        # Set the update rate
        rospy.Timer(rospy.Duration(.05), self.timer_callback) # 20hz

        self.tf_pub = tf2_ros.TransformBroadcaster()

        # Set subscribers
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.sub_robot_pose_update)

    def sub_robot_pose_update(self, msg):
        try:
            index = msg.name.index(f'{robot_name}::{robot_name}/base_footprint')
        except ValueError:
            # Do not fabricate stamped identity odometry before the car exists.
            return
        stamp = rospy.Time.now()
        pose = copy.deepcopy(msg.pose[index])
        twist = copy.deepcopy(msg.twist[index])
        q = pose.orientation
        inverse = tf.transformations.quaternion_inverse([q.x, q.y, q.z, q.w])
        transform = TransformStamped()
        transform.transform.rotation.x, transform.transform.rotation.y = inverse[:2]
        transform.transform.rotation.z, transform.transform.rotation.w = inverse[2:]
        # Both Twist vectors belong to child_frame_id, not the Gazebo world.
        # Convert local copies before exposing a new snapshot to the timer.
        for field in ('linear', 'angular'):
            vector = Vector3Stamped()
            vector.vector = getattr(twist, field)
            converted = tf2_geometry_msgs.do_transform_vector3(vector, transform)
            setattr(twist, field, converted.vector)
        with self.lock:
            self.last_received_pose = pose
            self.last_received_twist = twist
            self.last_recieved_stamp = stamp

    def timer_callback(self, event):
        with self.lock:
            if self.last_recieved_stamp is None:
                return
            pose = copy.deepcopy(self.last_received_pose)
            twist = copy.deepcopy(self.last_received_twist)
            stamp = self.last_recieved_stamp

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = f'{robot_name}/odom'
        odom.child_frame_id = f'{robot_name}/base_footprint'
        odom.pose.pose = pose
        odom.twist.twist = twist
        odom.pose.covariance =[1e-3, 0, 0, 0, 0, 0,
						0, 1e-3, 0, 0, 0, 0,
						0, 0, 1e6, 0, 0, 0,
						0, 0, 0, 1e6, 0, 0,
						0, 0, 0, 0, 1e6, 0,
						0, 0, 0, 0, 0, 1e3]

        odom.twist.covariance = [1e-9, 0, 0, 0, 0, 0, 
                          0, 1e-3, 1e-9, 0, 0, 0,
                          0, 0, 1e6, 0, 0, 0,
                          0, 0, 0, 1e6, 0, 0,
                          0, 0, 0, 0, 1e6, 0,
                          0, 0, 0, 0, 0, 1e-9]


        self.pub_odom.publish(odom)

        tf = TransformStamped(
            header=Header(
                frame_id=odom.header.frame_id,
                stamp=odom.header.stamp
            ),
            child_frame_id=odom.child_frame_id,
            transform=Transform(
                translation=odom.pose.pose.position,
                rotation=odom.pose.pose.orientation
            )
        )
        self.tf_pub.sendTransform(tf)

# Start the node
if __name__ == '__main__':
    rospy.init_node("gazebo_odometry_node")
    node = OdometryNode()
    rospy.spin()
