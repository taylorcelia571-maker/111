#!/usr/bin/env python3
"""Track a closed path using localization, odometry and live LaserScan."""
import json
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
    def __init__(self, path=None, clearance_guard=None):
        self.clearance_guard = clearance_guard
        self.generated = path is not None
        if path is None:
            filename = os.path.expanduser(rospy.get_param('~path_file'))
            path = ClosedPath(np.loadtxt(filename, delimiter=',', comments='#'))
        self.path = path
        follower_type = LineFollower
        if self.generated:
            from predictive_follower import PredictiveFollower
            follower_type = PredictiveFollower
        defaults = (('max_speed', 2.0), ('lateral_accel', 5.0), ('acceleration', 2.0),
                    ('braking', 3.0), ('max_error', 0.15)) if self.generated else (
                    ('max_speed', 3.0), ('lateral_accel', 3.0), ('acceleration', 2.5),
                    ('braking', 3.5), ('max_error', 0.6))
        options = {k: rospy.get_param('~' + k, v) for k,v in defaults}
        if self.generated: options['fast_start'] = rospy.get_param('~fast_start',False)
        self.follower = follower_type(self.path, **options)
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
        self.status_service = rospy.Service('~status', Trigger, self.status)
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
        if now.to_sec() <= 0:
            raise ValueError('Waiting for initialized ROS clock; start/unpause simulation')
        for label,msg,received in (('scan',self.scan,self.scan_time),('odom',self.odom,self.odom_time)):
            if msg is None or received is None:
                raise ValueError('%s missing on %s' % (label,rospy.resolve_name(label)))
            received_age = (now-received).to_sec()
            stamp_age = (now-msg.header.stamp).to_sec()
            # Separate topics can arrive just ahead of /clock over ROS transport.
            # Allow only 20ms future skew; keep the 300ms stale-data limit.
            if not -0.02 <= received_age <= 0.3 or not -0.02 <= stamp_age <= 0.3 or msg.header.stamp.to_sec() <= 0:
                raise ValueError('%s timing invalid: now=%.6f stamp=%.6f stamp_age=%.6fs received_age=%.6fs '
                                 '(allowed -0.020..0.300s); check /clock and topic publisher' %
                                 (label,now.to_sec(),msg.header.stamp.to_sec(),stamp_age,received_age))
        stamp = self.listener.getLatestCommonTime(self.frame, self.base)
        age = (now-stamp).to_sec()
        if not -0.02 <= age <= 0.3:
            raise ValueError('Localization TF timing invalid: age=%.6fs (allowed -0.020..0.300s)' % age)
        position, rotation = self.listener.lookupTransform(self.frame, self.base, rospy.Time(0))
        yaw = tf.transformations.euler_from_quaternion(rotation)[2]
        if not all(math.isfinite(v) for v in (*position[:2], yaw, self.odom.twist.twist.linear.x)):
            raise ValueError('Nonfinite localization or odometry')
        offset = 0.13 if self.generated else 0.0
        return (position[0]-offset*math.cos(yaw), position[1]-offset*math.sin(yaw),
                yaw, self.odom.twist.twist.linear.x)

    def status(self, request):
        with self.lock:
            try:
                pose = self.sensors()
                return TriggerResponse(True,json.dumps(dict(active=self.active,pose=pose,
                                                            message='Feedback ready; start still checks controller conflicts and pose')))
            except (ValueError,tf.Exception) as error:
                return TriggerResponse(False,str(error))

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
                rospy.loginfo('Race start: rear_pose=(%.4f,%.4f,%.4f) measured_v=%.4f progress=%.4fm',
                              *pose, self.follower.progress)
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
                        now = rospy.Time.now()
                        dt = (now - self.previous).to_sec()
                        if not math.isfinite(dt) or dt < 0:
                            raise ValueError('Control clock invalid/reset: dt=%.6fs' % dt)
                        # /clock can repeat while paused or between Gazebo updates.
                        # Do not integrate, advance progress, or latch a fault at dt=0.
                        if dt > 0:
                            self.control_cycle(now, dt)
                    except (ValueError, tf.Exception) as error:
                        self.stop(None)
                        rospy.logerr('Race stopped: %s', error)
            try:
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                self.stop(None)
                rate = rospy.Rate(50)

    def control_cycle(self, now, dt):
        pose = self.sensors()
        maximum_gap = 0.3 if self.generated else 0.2
        if dt > maximum_gap:
            raise ValueError('Control scheduling gap: dt=%.6fs, limit=%.3fs' % (dt, maximum_gap))
        self.previous = now
        if self.clearance_guard is not None:
            self.clearance_guard(pose)
        if self.generated:
            # Fresh feedback is mandatory. Project over the actual
            # elapsed arc, but never catch up acceleration/steering
            # by integrating an entire delayed interval at once.
            speed, steer = self.follower.step(*pose, min(dt, 0.04), projection_dt=dt)
            if dt > 0.1:
                rospy.logwarn_throttle(1, 'Control scheduling delay: dt=%.6fs; fresh feedback accepted', dt)
        else:
            speed, steer = self.follower.step(*pose, dt)
        if self.generated:
            from predictive_follower import reference_clearance
            # A scan is already allowed to be up to 300ms old. Transform
            # its points at acquisition time, not at the latest car pose.
            if not self.listener.canTransform(self.frame, self.scan.header.frame_id,
                                              self.scan.header.stamp):
                # Allow bounded transport skew between scan and odometry TF.
                self.listener.waitForTransform(self.frame, self.scan.header.frame_id,
                                               self.scan.header.stamp, rospy.Duration(0.05),
                                               rospy.Duration(0.005))
            laser_pos, laser_rot = self.listener.lookupTransform(
                self.frame, self.scan.header.frame_id, self.scan.header.stamp)
            laser_yaw = tf.transformations.euler_from_quaternion(laser_rot)[2]
            dx,dy = laser_pos[0]-pose[0],laser_pos[1]-pose[1]
            relative_laser = (math.cos(pose[2])*dx+math.sin(pose[2])*dy,
                              -math.sin(pose[2])*dx+math.cos(pose[2])*dy,
                              laser_yaw-pose[2])
            report = {}
            obstacle_limit = reference_clearance(
                self.path, self.follower.progress, pose, self.scan,
                relative_laser, max(speed, abs(pose[3])),
                self.follower.braking, diagnostics=report)
            report.update(scan_age=(now-self.scan.header.stamp).to_sec(),
                          speed_limit=obstacle_limit if math.isfinite(obstacle_limit) else None,
                          tracking_error=self.follower.error)
            if obstacle_limit < speed:
                rospy.logwarn_throttle(1, 'Laser speed restriction: %s', json.dumps(report,sort_keys=True))
            if obstacle_limit < 0.05:
                raise ValueError('Laser protection: '+json.dumps(report,sort_keys=True))
        else:
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

if __name__ == '__main__':
    rospy.init_node('continuous_race')
    try:
        ContinuousRace().run()
    except (ValueError, OSError) as error:
        rospy.logfatal('Cannot initialize continuous race: %s', error)
        raise SystemExit(1)
