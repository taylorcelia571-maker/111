#!/usr/bin/env python3
"""Persistent ROS publishers with optional Ackermann rolling feedforward."""
import threading
import rospy
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped
from wheel_kinematics import wheel_commands


class ServoCommands:
    def __init__(self):
        self.lock = threading.RLock()
        self.braking = False
        self.feedforward = rospy.get_param('~wheel_feedforward', False)
        topics = {'lr': 'left_rear_wheel_velocity_controller/command',
                  'rr': 'right_rear_wheel_velocity_controller/command',
                  'lf': 'left_front_wheel_velocity_controller/command',
                  'rf': 'right_front_wheel_velocity_controller/command',
                  'left': 'left_steering_hinge_position_controller/command',
                  'right': 'right_steering_hinge_position_controller/command'}
        self.publishers = {key: rospy.Publisher(topic, Float64, queue_size=1)
                           for key, topic in topics.items()}
        self.subscriber = rospy.Subscriber('ackermann_cmd_stamped', AckermannDriveStamped,
                                           self.command, queue_size=1)
        self.service = rospy.Service('emergency_brake', Empty, self.emergency_brake)
        rospy.on_shutdown(self.stop)

    def publish(self, commands):
        for key, publisher in self.publishers.items(): publisher.publish(commands[key])

    def command(self, msg):
        with self.lock:
            values = wheel_commands(0.0, 0.0) if self.braking else wheel_commands(
                msg.drive.speed, msg.drive.steering_angle, self.feedforward)
            self.publish(values)

    def emergency_brake(self, request):
        with self.lock:
            self.braking = not self.braking
            if self.braking: self.publish(wheel_commands(0.0, 0.0))
            rospy.logwarn('Emergency brake %s', 'activated' if self.braking else 'deactivated')
            return EmptyResponse()

    def stop(self):
        with self.lock:
            self.braking = True
            self.publish(wheel_commands(0.0, 0.0))


if __name__ == '__main__':
    rospy.init_node('servo_commands', anonymous=True)
    servo = ServoCommands()
    rospy.spin()
