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
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Log
from motion_metrics import summarize


def main():
    rospy.init_node('race_diagnostics')
    duration = float(rospy.get_param('~duration', 10.0))
    if not 1 <= duration <= 300: raise ValueError('duration must be 1..300 wall seconds')
    output = os.path.expanduser(rospy.get_param('~output', '~/ican_race_logs/motion_report.json'))
    lock = threading.Lock()
    data = {}
    samples = {}
    started = time.monotonic()

    def save(name, value):
        with lock:
            item = data.setdefault(name, {'count': 0, 'nonzero_count': 0})
            item['count'] += 1
            item['latest'] = value
            rows = samples.setdefault(name, [])
            if len(rows) < 16000: rows.append(dict(value, t=rospy.Time.now().to_sec()))
            item['last_received_wall_s'] = round(time.monotonic() - started, 3)
            if abs(value.get('speed', 0)) > 0.02: item['nonzero_count'] += 1

    def joint_sample(msg):
        positions = dict(zip(msg.name, msg.position))
        left = positions.get('left_steering_hinge_joint')
        right = positions.get('right_steering_hinge_joint')
        if left is not None and right is not None: save('joints', {'left': left, 'right': right})

    def planner_log(msg):
        if msg.name.rstrip('/') == rospy.get_namespace().rstrip('/')+'/move_base' and 'trajectory is not feasible' in msg.msg:
            save('teb_resets', {'kind': 'trajectory_infeasible'})
        if msg.name.rstrip('/') == rospy.get_namespace().rstrip('/')+'/move_base':
            if 'Control loop missed' in msg.msg: save('control_deadlines', {'kind': 'control_loop_overrun'})
            if 'Map update loop missed' in msg.msg: save('map_deadlines', {'kind': 'map_loop_overrun'})

    subs = [rospy.Subscriber('/rosout_agg', Log, planner_log, queue_size=100), rospy.Subscriber('joint_states', JointState, joint_sample, queue_size=10),
        rospy.Subscriber('cmd_vel', Twist, lambda m: save('cmd_vel',
            {'speed': m.linear.x, 'steer': m.angular.z}), queue_size=10),
        rospy.Subscriber('ackermann_cmd_stamped', AckermannDriveStamped, lambda m: save('ackermann',
            {'speed': m.drive.speed, 'steer': m.drive.steering_angle}), queue_size=10),
        rospy.Subscriber('odom', Odometry, lambda m: save('odom',
            {'speed': m.twist.twist.linear.x, 'x': m.pose.pose.position.x,
             'y': m.pose.pose.position.y}), queue_size=10),
        rospy.Subscriber('move_base/status', GoalStatusArray, lambda m: save('goal_status',
            {'statuses': [g.status for g in m.status_list]}), queue_size=1)]
    namespace = rospy.get_namespace()
    settings = {key: rospy.get_param(namespace+'move_base/'+key, None) for key in (
        'base_global_planner', 'controller_frequency', 'planner_frequency', 'TebLocalPlannerROS/max_vel_x',
        'TebLocalPlannerROS/max_vel_trans', 'TebLocalPlannerROS/use_proportional_saturation',
        'TebLocalPlannerROS/min_turning_radius', 'local_costmap/width', 'local_costmap/height',
        'TebLocalPlannerROS/max_vel_theta', 'TebLocalPlannerROS/wheelbase',
        'TebLocalPlannerROS/max_global_plan_lookahead_dist', 'TebLocalPlannerROS/global_plan_viapoint_sep')}
    settings['wheel_feedforward'] = rospy.get_param(namespace+'servo_commands/wheel_feedforward', False)
    for key in ('corner_speed_control', 'straight_speed', 'corner_speed'):
        settings[key] = rospy.get_param(namespace+'nav_sim/'+key, None)
    sim_start = rospy.Time.now().to_sec()
    while not rospy.is_shutdown() and time.monotonic() - started < duration:
        time.sleep(0.1)
    with lock:
        report = {'wall_duration_s': duration,
                  'simulation_elapsed_s': rospy.Time.now().to_sec() - sim_start,
                  'namespace': namespace, 'settings': settings, 'topics': data,
                  'metrics': summarize(samples, settings.get('TebLocalPlannerROS/max_vel_x') or 1.8,
                      settings.get('TebLocalPlannerROS/max_vel_theta') or 3.6,
                      0.26 if settings['wheel_feedforward'] else 0.265),
                  'samples': samples}
    os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
    with open(output, 'w') as stream: json.dump(report, stream, indent=2, ensure_ascii=False)
    print(json.dumps({key: value for key, value in report.items() if key != 'samples'},
                     indent=2, ensure_ascii=False))
    print('Saved: ' + output)
    for subscriber in subs: subscriber.unregister()


if __name__ == '__main__':
    try:
        main()
    except (ValueError, OSError) as error:
        print('Diagnostics failed: ' + str(error))
        raise SystemExit(1)
