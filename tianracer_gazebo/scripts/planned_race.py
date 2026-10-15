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
                          margin=rospy.get_param("~route_margin",0.26),
                          control_scoring=rospy.get_param('~route_control_scoring',False),
                          steering_preview_time=rospy.get_param('~steering_preview_time',.10),
                          steering_feedback_gain=rospy.get_param('~steering_feedback_gain',.25))
    if planner.control_scoring and rospy.get_param('~lateral_control','pursuit') != 'race_feedback':
        raise ValueError('Controller route score requires race_feedback')
    rospy.loginfo('Planning full course while stationary; no motion until start service')
    optimize = rospy.get_param('~optimize_time',False)
    center = planner.build(anchors,line_search_level=rospy.get_param('~line_search_level',0),speed_planning=rospy.get_param("~speed_planning","legacy"),optimize_time=optimize,refine_line=rospy.get_param('~refine_line',False),
                           **{k:rospy.get_param('~'+k,v) for k,v in (
                               ('max_speed',2.0),('lateral_accel',5.0),('acceleration',2.0),('braking',3.0),
                               ('steering_rate',2.0),('yaw_rate',3.2),('response_delay',0.0))})
    if optimize:
        rospy.loginfo('Time-based route selection (model only): %s',planner.selection)
    unpolished_center = center
    polish_report = None
    if rospy.get_param('~route_joint_polish',False):
        if not optimize:
            raise ValueError('Joint line polish requires optimize_time')
        from joint_line_polish import polish_line
        center,polish_report=polish_line(planner,center,anchors,
            {k:rospy.get_param('~'+k,v) for k,v in (
                ('max_speed',2.0),('lateral_accel',5.0),('acceleration',2.0),('braking',3.0),
                ('steering_rate',2.0),('yaw_rate',3.2),('response_delay',.25),('speed_planning','legacy'))},
            strength=rospy.get_param('~route_polish_strength',.5))
        planner.selection.update(joint_polish=polish_report,
            estimated_lap_seconds=polish_report['selected_model_lap_seconds'])
        if polish_report['accepted']:
            planner.selection.update(route_changed=True,route_sha256=polish_report['selected_route_sha256'])
        rospy.loginfo('Joint route polish (model only): %s',polish_report)
    speed_refine_report = None
    if rospy.get_param('~route_speed_refine',False):
        if not optimize:
            raise ValueError('Speed-aware route refinement requires optimize_time')
        from speed_aware_line import refine_speed_line
        center,speed_refine_report=refine_speed_line(planner,center,anchors,
            {k:rospy.get_param('~'+k,v) for k,v in (
                ('max_speed',2.0),('lateral_accel',5.0),('acceleration',2.0),('braking',3.0),
                ('steering_rate',2.0),('yaw_rate',3.2),('response_delay',.25),('speed_planning','legacy'))},
            iterations=rospy.get_param('~route_speed_iterations',6), source_seed=unpolished_center)
        planner.selection.update(speed_refinement=speed_refine_report,
            estimated_lap_seconds=speed_refine_report['selected_model_lap_seconds'])
        if speed_refine_report['accepted']:
            planner.selection.update(route_changed=True,route_sha256=speed_refine_report['selected_route_sha256'])
        rospy.loginfo('Speed-aware route refinement (model only): %s',speed_refine_report)
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
    controller.route_polish_report=polish_report
    controller.route_speed_refine_report=speed_refine_report
    # Persist the plan for inspection, never as an official result.
    output = os.path.expanduser(rospy.get_param('~output','~/ican_race_logs/planned_course.csv'))
    os.makedirs(os.path.dirname(os.path.abspath(output)),exist_ok=True)
    np.savetxt(output,np.column_stack([path.points,path.curvature,path.speeds]),delimiter=',',
               header='rear_axle_x,rear_axle_y,curvature,speed_limit')
    report=controller.follower.plan_report()
    report.update(route_selection=planner.selection,official_lap_validation_required=True,
                  route_margin=planner.margin,wall_stop_clearance=.23,
                  nominal_wall_tracking_reserve=planner.margin-.23)
    report['route_control_scoring']=planner.control_scoring
    report['route_joint_polish']=polish_report
    report['route_speed_refine']=speed_refine_report
    from speed_aware_line import sector_speed_report
    report['sector_speed_planning']=sector_speed_report(path)
    report['route_control_score_assumptions']= (
        'Perfect localization and current-reference steering, zero joint age; '
        'feedforward/compensation/slew proxy only, not actuator or tire validation'
        if planner.control_scoring else None)
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
