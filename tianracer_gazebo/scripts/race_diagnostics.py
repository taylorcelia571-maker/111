#!/usr/bin/env python3
"""Read-only, wall-time bounded motion-chain report; no vehicle commands."""
import json
import os
import threading
import time
import rospy
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatusArray


def main():
    rospy.init_node('race_diagnostics')
    duration = float(rospy.get_param('~duration', 10.0))
    if not 1 <= duration <= 60: raise ValueError('duration must be 1..60 wall seconds')
    output = os.path.expanduser(rospy.get_param('~output', '~/ican_race_logs/motion_report.json'))
    lock = threading.Lock()
    data = {}
    started = time.monotonic()

    def save(name, value):
        with lock:
            item = data.setdefault(name, {'count': 0, 'nonzero_count': 0})
            item['count'] += 1
            item['latest'] = value
            item['last_received_wall_s'] = round(time.monotonic() - started, 3)
            if abs(value.get('speed', 0)) > 0.02: item['nonzero_count'] += 1

    subs = [
        rospy.Subscriber('cmd_vel', Twist, lambda m: save('cmd_vel',
            {'speed': m.linear.x, 'steer': m.angular.z}), queue_size=10),
        rospy.Subscriber('ackermann_cmd_stamped', AckermannDriveStamped, lambda m: save('ackermann',
            {'speed': m.drive.speed, 'steer': m.drive.steering_angle}), queue_size=10),
        rospy.Subscriber('odom', Odometry, lambda m: save('odom',
            {'speed': m.twist.twist.linear.x, 'x': m.pose.pose.position.x,
             'y': m.pose.pose.position.y}), queue_size=10),
        rospy.Subscriber('move_base/status', GoalStatusArray, lambda m: save('goal_status',
            {'statuses': [g.status for g in m.status_list]}), queue_size=1)]
    sim_start = rospy.Time.now().to_sec()
    while not rospy.is_shutdown() and time.monotonic() - started < duration:
        time.sleep(0.1)
    with lock:
        report = {'wall_duration_s': duration,
                  'simulation_elapsed_s': rospy.Time.now().to_sec() - sim_start,
                  'namespace': rospy.get_namespace(), 'topics': data}
    os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
    with open(output, 'w') as stream: json.dump(report, stream, indent=2, ensure_ascii=False)
    print(json.dumps(report, indent=2, ensure_ascii=False))
    print('Saved: ' + output)
    for subscriber in subs: subscriber.unregister()


if __name__ == '__main__':
    try:
        main()
    except (ValueError, OSError) as error:
        print('Diagnostics failed: ' + str(error))
        raise SystemExit(1)
