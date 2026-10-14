#!/usr/bin/env python3
"""User71 hypothetical five-loop comparisons; never starts ROS or a referee."""
import argparse
import json
import math
from pathlib import Path
import time
import numpy as np
from race60_audit import load_course, rear_path
from predictive_follower import PredictiveFollower


BUDGETS=dict(max_speed=10.,lateral_accel=9.94,acceleration=7.3,braking=3.7,
             yaw_rate=4.4,steering_rate=2.)
OPTIONS=dict(response_delay=.25,fast_start=True,start_blend=True,
    lateral_control='race_feedback',steering_feedback_gain=.25,
    startup_release_early=True,startup_speed=2.,startup_acceleration=2.5,
    steering_preview_time=.12,speed_planning='delayed_braking',
    heading_feedback_gain=.8,cross_track_gain=.8)


def simulate(center,planner,options,lag,pose_age,joint_age,budgets=None):
    budgets=BUDGETS if budgets is None else budgets
    follower=PredictiveFollower(rear_path(center),**budgets,**options)
    x,y,yaw=.0081,-.1665,1.5707
    speed=steer=peak=actual_peak=0.;wall=math.inf;loops=[];release=None
    pose_history=[(x,y,yaw,speed,steer)]*(round(pose_age/.02)+1)
    joint_history=[steer]*(round(joint_age/.02)+1)
    follower.start(x,y,yaw)
    row=dict(official_result=False,hypothetical_actuator_lag=lag,
        pose_age=pose_age,joint_transport_age=joint_age,
        reference_sha256=follower.path.fingerprint(),options=options,budgets=budgets)
    max_drop=0.;old_command=0.
    try:
        for tick in range(12000):
            elapsed=(tick+1)*.02;hx,hy,hyaw,hv,hd=pose_history[0]
            if joint_age>.3:raise ValueError('Steering feedback stale (adapter guard)')
            follower.measured_steer=joint_history[0];follower.measured_steer_age=joint_age
            v,d=follower.step(hx,hy,hyaw,hv,.02)
            max_drop=max(max_drop,old_command-v);old_command=v
            speed+=(v-speed)*.02/lag;steer+=(d-steer)*.02/lag
            x+=speed*math.cos(yaw)*.02;y+=speed*math.sin(yaw)*.02
            yaw+=speed*math.tan(steer)/.26*.02
            pose_history.append((x,y,yaw,speed,steer));pose_history.pop(0)
            joint_history.append(steer);joint_history.pop(0)
            peak=max(peak,follower.error)
            actual_peak=max(actual_peak,follower.path.project(x,y,follower.progress-.9,follower.progress+1.)[1])
            wall=min(wall,planner.check_vehicle_clearance((x,y,yaw,speed)))
            if actual_peak>.15:raise ValueError('Actual model tracking corridor exceeded')
            if release is None and not follower.launch_limited and not follower.departure:release=elapsed
            completed=(follower.progress-follower.origin)/follower.path.length
            if completed>=len(loops)+1:loops.append(elapsed)
            if completed>=5:
                row.update(completed=True,total_seconds=elapsed,first_loop_seconds=loops[0],
                    peak_reference_error=peak,peak_actual_error=actual_peak,
                    minimum_center_clearance=wall,startup_release_seconds=release,
                    maximum_single_cycle_command_drop=max_drop)
                return row
        raise ValueError('Model timeout')
    except ValueError as error:
        row.update(completed=False,fault=str(error),elapsed=elapsed,
            peak_reference_error=peak,peak_actual_error=actual_peak,
            minimum_center_clearance=wall,startup_release_seconds=release,
            maximum_single_cycle_command_drop=max_drop)
        return row


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--quick',action='store_true')
    args=parser.parse_args()
    package=Path(__file__).resolve().parents[1]
    routes={};rows=[];planning=[]
    for enabled in (False,True):
        planner,anchors=load_course(package)
        planner.margin=.34;planner.allowed=planner.clearance>=.34
        planner.control_scoring=enabled;planner.scoring_preview_time=.12;planner.scoring_feedback_gain=.25
        started=time.perf_counter()
        center=planner.build(anchors,optimize_time=True,refine_line=True,line_search_level=3,
            response_delay=.25,speed_planning='delayed_braking',**BUDGETS)
        routes[enabled]=(planner,center)
        item=dict(controller_score=enabled,wall_seconds=time.perf_counter()-started,
            selection=dict(planner.selection),minimum_route_clearance=planner.segment_check(center.points))
        planning.append(item);print(json.dumps(item),flush=True)
    cases=[('baseline',False,{}),('v1_connector24',False,{'startup_connector_length':2.4}),
           ('v1_speed21',False,{'startup_speed':2.1}),
           ('v2_route_score',True,{}),('v3_connector24_route_score',True,{'startup_connector_length':2.4})]
    disturbances=[(.08,.02,0.),(.15,.02,0.),(.15,.04,.04),(.25,.04,.04),(.15,.02,.12)]
    if args.quick:disturbances=disturbances[:2]
    for name,scoring,change in cases:
        planner,center=routes[scoring]
        options=dict(OPTIONS,**change)
        for lag,age,joint in disturbances:
            row=simulate(center,planner,options,lag,age,joint);row['name']=name
            rows.append(row);print(json.dumps(row),flush=True)
    output=args.output.expanduser();output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(dict(official_result=False,gazebo_tested=False,
        official_under_60_verified=False,user_reported_best_seconds=71,
        assumptions='Bicycle plus first-order actuator response; no tires/roll/AMCL/laser/referee. Transport age includes sampled history approximation.',
        planning=planning,rows=rows),indent=2)+'\n')


if __name__=='__main__':main()
