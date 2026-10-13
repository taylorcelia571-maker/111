#!/usr/bin/env python3
"""Generate a collision-checked closed course once; track it without move_base."""
import json
import math
import os
import numpy as np
import yaml
import rospy
import rospkg
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from loop_planner import LoopPlanner
from race_line import ClosedPath
from continuous_race import ContinuousRace


def main():
    rospy.init_node('continuous_race')
    grid = rospy.wait_for_message('map', OccupancyGrid, timeout=60)
    if grid.header.frame_id != rospy.get_param('~map_frame','map'):
        raise ValueError('Map frame mismatch')
    q = grid.info.origin.orientation
    if abs(q.x)+abs(q.y)+abs(q.z)>1e-6 or abs(q.w-1)>1e-6:
        raise ValueError('Rotated occupancy grids are unsupported')
    package = rospkg.RosPack().get_path('tianracer_gazebo')
    filename = rospy.get_param('~waypoints', os.path.join(package,'scripts/waypoint_race/tianracer_racetrack_points.yaml'))
    with open(filename) as stream: data=yaml.safe_load(stream)
    if any(w['frame_id'] != grid.header.frame_id for w in data['waypoints']):
        raise ValueError('Waypoint frame mismatch')
    anchors = [[w['pose']['position'][k] for k in ('x','y')] for w in data['waypoints']]
    occupancy = np.asarray(grid.data).reshape(grid.info.height,grid.info.width)
    planner = LoopPlanner(occupancy,grid.info.resolution,
                          [grid.info.origin.position.x,grid.info.origin.position.y],
                          margin=rospy.get_param("~route_margin",0.26))
    rospy.loginfo('Planning full course while stationary; no motion until start service')
    optimize = rospy.get_param('~optimize_time',False)
    center = planner.build(anchors,line_search_level=rospy.get_param('~line_search_level',0),speed_planning=rospy.get_param("~speed_planning","legacy"),optimize_time=optimize,refine_line=rospy.get_param('~refine_line',False),
                           **{k:rospy.get_param('~'+k,v) for k,v in (
                               ('max_speed',2.0),('lateral_accel',5.0),('acceleration',2.0),('braking',3.0),
                               ('steering_rate',2.0),('yaw_rate',3.2),('response_delay',0.0))})
    if optimize:
        rospy.loginfo('Time-based route selection (model only): %s',planner.selection)
    tangent = np.roll(center.points,-1,axis=0)-np.roll(center.points,1,axis=0)
    tangent /= np.linalg.norm(tangent,axis=1)[:,None]
    rear = center.points-0.13*tangent
    path = ClosedPath(np.vstack([rear,rear[0]]),spacing=0.04)
    def guard(pose):
        planner.check_vehicle_clearance(pose)
    def start_guard(follower):
        arcs=np.linspace(0,follower.connector_length,100)+follower.origin
        centers=[]
        for distance in arcs:
            tangent=follower.sample_tracking(distance+.02)-follower.sample_tracking(distance-.02)
            tangent/=np.linalg.norm(tangent)
            centers.append(follower.sample_tracking(distance)+.13*tangent)
        return bool(np.min(planner.clearance_at(np.asarray(centers)))>=planner.margin)
    controller = ContinuousRace(path, clearance_guard=guard,start_path_guard=start_guard)
    # Persist the plan for inspection, never as an official result.
    output = os.path.expanduser(rospy.get_param('~output','~/ican_race_logs/planned_course.csv'))
    os.makedirs(os.path.dirname(os.path.abspath(output)),exist_ok=True)
    np.savetxt(output,np.column_stack([path.points,path.curvature,path.speeds]),delimiter=',',
               header='rear_axle_x,rear_axle_y,curvature,speed_limit')
    report=controller.follower.plan_report()
    report.update(route_selection=planner.selection,official_lap_validation_required=True,
                  route_margin=planner.margin,wall_stop_clearance=.23,
                  nominal_wall_tracking_reserve=planner.margin-.23)
    report_file=os.path.splitext(output)[0]+'_report.json'
    with open(report_file,'w') as stream: json.dump(report,stream,indent=2,sort_keys=True)
    rospy.loginfo('Speed budget report (model, not result): %s; saved %s',json.dumps(report,sort_keys=True),report_file)
    if not report['speed_ceiling_binding']:
        rospy.loginfo('max_speed=%.2f is not binding; peak planned speed=%.2f. '
                      'See same_route_parameter_sensitivity before increasing the ceiling.',
                      controller.follower.max_speed,report['peak_planned_speed'])
    preview = Path(); preview.header.frame_id=grid.header.frame_id
    preview.header.stamp=rospy.Time.now()
    for i,p in enumerate(path.points):
        msg=PoseStamped(); msg.header=preview.header
        msg.pose.position.x,msg.pose.position.y=p
        d=path.points[(i+1)%path.n]-p; yaw=math.atan2(d[1],d[0])
        msg.pose.orientation.z=math.sin(yaw/2); msg.pose.orientation.w=math.cos(yaw/2)
        preview.poses.append(msg)
    publisher=rospy.Publisher('race_path',Path,queue_size=1,latch=True)
    publisher.publish(preview)
    baseline=Path();baseline.header=preview.header
    points=getattr(planner,'baseline_points',center.points)
    tangent=np.roll(points,-1,axis=0)-np.roll(points,1,axis=0)
    tangent/=np.linalg.norm(tangent,axis=1)[:,None]
    for p in points-.13*tangent:
        msg=PoseStamped();msg.header=baseline.header
        msg.pose.position.x,msg.pose.position.y=p;msg.pose.orientation.w=1
        baseline.poses.append(msg)
    baseline_publisher=rospy.Publisher('race_baseline',Path,queue_size=1,latch=True)
    baseline_publisher.publish(baseline)
    rospy.loginfo('Ready: %.2fm, center clearance >=%.3fm, max curvature %.3f/m; plan: %s',
                  path.length,planner.segment_check(center.points),np.max(np.abs(path.curvature)),output)
    controller.run()


if __name__=='__main__':
    try: main()
    except (ValueError,OSError,rospy.ROSException) as error:
        rospy.logfatal('Planned race initialization failed: %s',error)
        raise SystemExit(1)
