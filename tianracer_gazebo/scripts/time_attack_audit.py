#!/usr/bin/env python3
"""Compare the user's 87.546s configuration and candidate in hypothetical models."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
from race60_audit import load_course, rear_path
from predictive_follower import PredictiveFollower

BASE=dict(max_speed=10,lateral_accel=6.5,acceleration=6.6,braking=3.6,yaw_rate=3.8,steering_rate=2)
FAST=dict(max_speed=10,lateral_accel=8,acceleration=8,braking=4.5,yaw_rate=4.5,steering_rate=2)


def build(budgets, mode):
    planner,anchors=load_course(Path(__file__).resolve().parents[1])
    planner.margin=.34;planner.allowed=planner.clearance>=.34
    center=planner.build(anchors,optimize_time=True,refine_line=True,response_delay=.25,
                         speed_planning=mode,**budgets)
    return planner,center


def evaluate(planner,center,budgets,lag,control,planning,early,joint_delay=0):
    f=PredictiveFollower(rear_path(center),response_delay=.25,fast_start=True,start_blend=True,
                         lateral_control=control,steering_feedback_gain=.25,speed_planning=planning,
                         startup_release_early=early,startup_speed=1.8,startup_acceleration=2,**budgets)
    x,y,yaw=.0081,-.1665,1.5707
    f.start(x,y,yaw);speed=steer=0;peak=0.;wall=1.;seams=[];release=None
    history=[0.]*(joint_delay+1);max_decel=0.;previous_command=0.
    row=dict(hypothetical_actuator_lag=lag,joint_transport_delay=joint_delay*.02,
             controller=control,speed_planning=planning,official_result=False,
             plan_five_seconds=float(5*np.sum(f.path.ds/f.preview_speed_profile())))
    try:
        for tick in range(12000):
            f.measured_steer=history[0];f.measured_steer_age=joint_delay*.02
            v,d=f.step(x,y,yaw,speed,.02)
            max_decel=max(max_decel,(previous_command-v)/.02);previous_command=v
            if release is None and not f.launch_limited:release=(tick+1)*.02
            speed+=(v-speed)*.02/lag;steer+=(d-steer)*.02/lag
            history.append(steer);history.pop(0)
            x+=speed*math.cos(yaw)*.02;y+=speed*math.sin(yaw)*.02
            yaw+=speed*math.tan(steer)/.26*.02
            peak=max(peak,f.error);wall=min(wall,planner.check_vehicle_clearance((x,y,yaw,speed)))
            loops=(f.progress-f.origin)/f.path.length
            if loops>=len(seams)+1:seams.append((tick+1)*.02)
            if loops>=5:
                row.update(completed=True,seconds=(tick+1)*.02,first_loop_seconds=seams[0],
                           loop_end_seconds=seams,peak_error=peak,minimum_center_clearance=wall,
                           startup_limit_released_seconds=release,maximum_command_deceleration=max_decel)
                return row
        raise ValueError('Model did not finish five loops')
    except ValueError as error:
        row.update(completed=False,fault=str(error),seconds=(tick+1)*.02,peak_error=peak)
        return row


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    result=dict(official_five_laps_under_60_verified=False,gazebo_tested=False,
                observed_user_official_seconds=87.546,profiles={})
    for name,budgets,mode,control,early in [
        ('user_baseline',BASE,'legacy','smooth_pursuit',False),
        ('new_same_budget',BASE,'delayed_braking','race_feedback',True),
        ('raised_candidate',FAST,'delayed_braking','race_feedback',True)]:
        planner,center=build(budgets,mode)
        rows=[evaluate(planner,center,budgets,lag,control,mode,early) for lag in (.08,.15,.2)]
        result['profiles'][name]=dict(budgets=budgets,rows=rows,
                                      route_length=rear_path(center).length,selection=planner.selection)
        print(name,json.dumps(rows),flush=True)
    args.output=args.output.expanduser();args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':main()
