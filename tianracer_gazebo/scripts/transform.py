#!/usr/bin/env python3
"""Convert TEB steering-angle Twist to Ackermann; optional timed race shaping."""
import math
import threading
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from race_motion import RaceMotion
from corner_speed import CornerSpeed, path_preview
from wheel_kinematics import equivalent_steer


class Converter:
    def __init__(self):
        self.robot_name = rospy.get_param('robot_name', 'tianracer')
        self.enabled = rospy.get_param('~race_profile', False)
        self.corner_enabled = rospy.get_param('~corner_speed_control', False)
        if self.corner_enabled and self.enabled:
            raise ValueError('Select corner speed control OR legacy race smoothing')
        self.timed = self.enabled or self.corner_enabled or rospy.get_param('~command_watchdog', False)
        self.pub = rospy.Publisher('ackermann_cmd_stamped', AckermannDriveStamped, queue_size=1)
        self.lock = threading.Lock()
        self.target = (0.0, 0.0)
        self.received = None
        self.previous = rospy.Time.now()
        self.timeout = rospy.get_param('~command_timeout', 0.5)
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('command_timeout must be finite and positive')
        self.motion = RaceMotion(**{key: rospy.get_param('~' + key, default) for key, default in (
            ('wheelbase', 0.265), ('max_speed', 2.4), ('max_steer', 0.48),
            ('lateral_accel', 2.5), ('acceleration', 2.5), ('deceleration', 4.0),
            ('steer_rate', 1.8), ('steer_tau', 0.06))})
        self.governor = None
        self.preview = self.joint_steer = None
        self.plan_received = self.joint_received = self.odom_received = None
        self.measured_speed = 0.0
        if self.corner_enabled:
            from nav_msgs.msg import Path, Odometry
            from sensor_msgs.msg import JointState
            self.governor = CornerSpeed(
                straight_speed=rospy.get_param('~straight_speed', 2.0),
                corner_speed=rospy.get_param('~corner_speed', 1.8))
            self.plan_sub = rospy.Subscriber('move_base/TebLocalPlannerROS/local_plan', Path,
                                             self.on_plan, queue_size=1)
            self.joint_sub = rospy.Subscriber('joint_states', JointState, self.on_joints, queue_size=1)
            self.odom_sub = rospy.Subscriber('odom', Odometry, self.on_odom, queue_size=1)
        self.sub = rospy.Subscriber('cmd_vel' , Twist, self.callback, queue_size=1)
        self.timer = None
        if self.timed:
            self.timer = rospy.Timer(rospy.Duration(0.02), self.tick, reset=True)
        rospy.on_shutdown(self.stop)

    def on_plan(self, msg):
        preview = path_preview([(p.pose.position.x, p.pose.position.y) for p in msg.poses])
        with self.lock:
            self.preview = preview
            self.plan_received = msg.header.stamp

    def on_joints(self, msg):
        positions = dict(zip(msg.name, msg.position))
        try:
            steer = equivalent_steer(positions['left_steering_hinge_joint'],
                                     positions['right_steering_hinge_joint'])
        except (KeyError, ValueError, ZeroDivisionError): return
        with self.lock:
            self.joint_steer = steer
            self.joint_received = msg.header.stamp

    def on_odom(self, msg):
        with self.lock:
            self.measured_speed = msg.twist.twist.linear.x
            self.odom_received = msg.header.stamp

    def publish(self, speed, steer):
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        prefix = self.robot_name.strip('/')
        msg.header.frame_id = (prefix + '/' if prefix else '') + 'base_footprint'
        msg.drive.speed = speed
        msg.drive.steering_angle = steer
        self.pub.publish(msg)

    def callback(self, data):
        speed, steer = data.linear.x, data.angular.z
        if not (math.isfinite(speed) and math.isfinite(steer)):
            rospy.logwarn_throttle(2.0, 'Invalid motion command; stopping')
            speed, steer = 0.0, 0.0
        with self.lock:
            if not self.timed:
                self.publish(speed, steer)
                return
            self.target = speed, steer
            self.received = rospy.Time.now()
            if speed == 0.0:
                self.motion.speed = 0.0
                if self.governor is not None: self.governor.speed = 0.0
                self.publish(0.0, self.motion.steer if self.enabled else steer)

    def tick(self, event):
        with self.lock:
            now = rospy.Time.now()
            dt = (now - self.previous).to_sec()
            self.previous = now
            age = (now - self.received).to_sec() if self.received is not None else -1
            if age < 0 or age > self.timeout or dt <= 0:
                if age < 0 or dt < 0:
                    self.received = None
                self.motion.speed = 0.0
                if self.governor is not None: self.governor.speed = 0.0
                self.publish(0.0, self.motion.steer)
                return
            if self.corner_enabled:
                def fresh(stamp): return stamp is not None and 0 <= (now-stamp).to_sec() <= 0.35
                if not fresh(self.odom_received):
                    self.governor.speed = 0.0
                    self.governor.reason = 'odom_stale'
                    rospy.logwarn_throttle(2.0, 'Speed guard stopped: odometry missing/stale (0.35 sim seconds)')
                    self.publish(0.0, self.target[1])
                    return
                preview = self.preview if fresh(self.plan_received) else None
                steer_actual = self.joint_steer if fresh(self.joint_received) else None
                speed = self.governor.step(*self.target, steer_actual, self.measured_speed, preview, dt)
                self.publish(speed, self.target[1])
                rospy.loginfo_throttle(2.0, 'Speed guard: requested=%.2f output=%.2f limit=%.2f reason=%s',
                                      self.target[0], speed, self.governor.last_limit, self.governor.reason)
            elif self.enabled:
                self.publish(*self.motion.step(*self.target, dt))
            else:
                self.publish(*self.target)

    def stop(self):
        if self.timer is not None:
            self.timer.shutdown()
        with self.lock:
            self.target = (0.0, 0.0)
            self.motion.speed = 0.0
            if self.governor is not None: self.governor.speed = 0.0
            self.publish(0.0, self.motion.steer)


if __name__ == '__main__':
    rospy.init_node('nav_sim', anonymous=True)
    try:
        converter = Converter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
