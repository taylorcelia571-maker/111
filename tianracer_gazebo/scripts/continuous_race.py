#!/usr/bin/env python3
"""Follow a recorded, verified full lap using localization and LaserScan only."""
import math
import os
import threading
import numpy as np
import rospy
import rosgraph
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger, TriggerResponse
from race_line import ClosedPath, LineFollower, clearance_speed


class ContinuousRace:
    def __init__(self):
        filename = os.path.expanduser(rospy.get_param('~path_file'))
        data = np.loadtxt(filename, delimiter=',', comments='#')
        self.path = ClosedPath(data)
        self.follower = LineFollower(self.path, **{k: rospy.get_param('~' + k, v) for k, v in (
            ('max_speed', 3.0), ('lateral_accel', 3.0), ('acceleration', 2.5),
            ('braking', 3.5), ('max_error', 0.6))})
        self.laps = rospy.get_param('~laps', 5)
        if isinstance(self.laps, bool) or not isinstance(self.laps, int) or self.laps < 1:
            raise ValueError('laps must be a positive integer')
        self.frame = rospy.get_param('~map_frame', 'map')
        robot = rospy.get_namespace().strip('/')
        self.base = rospy.get_param('~base_frame', robot + '/base_footprint')
        self.listener = tf.TransformListener()
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.lock = threading.RLock()
        self.scan = self.odom = None
        self.scan_time = self.odom_time = None
        self.active = False
        self.previous = rospy.Time.now()
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.on_scan, queue_size=1)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.on_odom, queue_size=1)
        self.start_service = rospy.Service('~start', Trigger, self.start)
        self.stop_service = rospy.Service('~stop', Trigger, self.stop)
        rospy.on_shutdown(lambda: self.stop(None))
        rospy.loginfo('Loaded %.1fm closed path. Idle; prepare referee then call ~start.', self.path.length)

    def on_scan(self, msg):
        with self.lock:
            self.scan, self.scan_time = msg, rospy.Time.now()

    def on_odom(self, msg):
        with self.lock:
            self.odom, self.odom_time = msg, rospy.Time.now()

    def sensors(self):
        now = rospy.Time.now()
        for msg, received in ((self.scan, self.scan_time), (self.odom, self.odom_time)):
            if msg is None or received is None or not 0 <= (now - received).to_sec() <= 0.3:
                raise ValueError('Scan or odometry missing/stale')
            if not 0 <= (now - msg.header.stamp).to_sec() <= 0.3:
                raise ValueError('Scan or odometry timestamp stale')
        stamp = self.listener.getLatestCommonTime(self.frame, self.base)
        if not 0 <= (now - stamp).to_sec() <= 0.3: raise ValueError('Localization TF stale')
        position, rotation = self.listener.lookupTransform(self.frame, self.base, rospy.Time(0))
        yaw = tf.transformations.euler_from_quaternion(rotation)[2]
        if not all(math.isfinite(v) for v in (*position[:2], yaw, self.odom.twist.twist.linear.x)):
            raise ValueError('Nonfinite localization or odometry')
        return position[0], position[1], yaw, self.odom.twist.twist.linear.x

    def start(self, request):
        with self.lock:
            if self.active: return TriggerResponse(False, 'Already active')
            try:
                publishers, _, _ = rosgraph.Master(rospy.get_name()).getSystemState()
                topic = rospy.resolve_name('cmd_vel')
                others = [name for published, names in publishers if published == topic
                          for name in names if name != rospy.get_name()]
                if others: raise ValueError('Other cmd_vel publishers active: ' + ', '.join(others))
                pose = self.sensors()
                if abs(pose[3]) > 0.05: raise ValueError('Vehicle must be stationary')
                self.follower.start(*pose[:3])
                self.follower.stop()
                self.previous = rospy.Time.now() - rospy.Duration(0.02)
                self.active = True
                return TriggerResponse(True, 'Started; official referee determines valid laps')
            except (ValueError, tf.Exception, rosgraph.MasterError, rosgraph.MasterFailure, OSError) as error:
                return TriggerResponse(False, str(error))

    def stop(self, request):
        with self.lock:
            self.active = False
            self.follower.stop()
            self.pub.publish(Twist())
            return TriggerResponse(True, 'Stopped')

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            with self.lock:
                if self.active:
                    try:
                        pose = self.sensors()
                        now = rospy.Time.now()
                        dt = (now - self.previous).to_sec()
                        self.previous = now
                        speed, steer = self.follower.step(*pose, dt)
                        laser_pos, laser_rot = self.listener.lookupTransform(
                            self.base, self.scan.header.frame_id, rospy.Time(0))
                        laser_yaw = tf.transformations.euler_from_quaternion(laser_rot)[2]
                        obstacle_limit = clearance_speed(
                            self.scan.ranges, self.scan.angle_min, self.scan.angle_increment,
                            self.scan.range_min, self.scan.range_max, max(speed, abs(pose[3])), steer,
                            braking=self.follower.braking,
                            laser_x=laser_pos[0], laser_y=laser_pos[1], laser_yaw=laser_yaw)
                        if obstacle_limit < 0.05:
                            raise ValueError('Obstacle/invalid scan: stopped; inspect then call start again')
                        speed = min(speed, obstacle_limit)
                        self.follower.speed = speed
                        progress = self.follower.progress - self.follower.origin
                        if progress >= self.laps * self.path.length:
                            self.stop(None)
                            rospy.loginfo('Reference loops complete; check OFFICIAL referee laps separately')
                        else:
                            msg = Twist()
                            msg.linear.x, msg.angular.z = speed, steer
                            self.pub.publish(msg)
                            rospy.loginfo_throttle(2, 'Reference %.2f/%d loops, v=%.2f, error=%.2fm',
                                                  progress / self.path.length, self.laps, speed, self.follower.error)
                    except (ValueError, tf.Exception) as error:
                        self.stop(None)
                        rospy.logerr('Race stopped: %s', error)
            try:
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                self.stop(None)
                rate = rospy.Rate(50)


if __name__ == '__main__':
    rospy.init_node('continuous_race')
    try:
        ContinuousRace().run()
    except (ValueError, OSError) as error:
        rospy.logfatal('Cannot initialize continuous race: %s', error)
        raise SystemExit(1)
