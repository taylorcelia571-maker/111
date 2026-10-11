#!/usr/bin/env python3
"""Convert TEB steering-angle Twist to Ackermann; optional timed race shaping."""
import math
import threading
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from race_motion import RaceMotion


class Converter:
    def __init__(self):
        self.robot_name = rospy.get_param('robot_name', 'tianracer')
        self.enabled = rospy.get_param('~race_profile', False)
        self.timed = self.enabled or rospy.get_param('~command_watchdog', False)
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
        self.sub = rospy.Subscriber('cmd_vel', Twist, self.callback, queue_size=1)
        self.timer = None
        if self.timed:
            self.timer = rospy.Timer(rospy.Duration(0.02), self.tick, reset=True)
        rospy.on_shutdown(self.stop)

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
                self.publish(0.0, self.motion.steer)
                return
            if self.enabled:
                self.publish(*self.motion.step(*self.target, dt))
            else:
                self.publish(*self.target)

    def stop(self):
        if self.timer is not None:
            self.timer.shutdown()
        with self.lock:
            self.target = (0.0, 0.0)
            self.motion.speed = 0.0
            self.publish(0.0, self.motion.steer)


if __name__ == '__main__':
    rospy.init_node('nav_sim', anonymous=True)
    try:
        converter = Converter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
