#!/usr/bin/env python3
"""Read-only idealized speed budgets, not a proven safe driving controller."""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np


def rotation(rpy):
    r,p,y=rpy
    cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                     [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])


def transform(origin):
    out=np.eye(4)
    if origin is not None:
        xyz=[float(v) for v in origin.attrib.get('xyz','0 0 0').split()]
        rpy=[float(v) for v in origin.attrib.get('rpy','0 0 0').split()]
        out[:3,3]=xyz;out[:3,:3]=rotation(rpy)
    return out


def model_com(filename):
    root=E.parse(filename).getroot()
    joints=root.findall('joint');links=root.findall('link')
    children={j.find('child').attrib['link'] for j in joints}
    roots=[l.attrib['name'] for l in links if l.attrib['name'] not in children]
    if len(roots)!=1:raise ValueError('Need one root and literal joint geometry')
    frames={roots[0]:np.eye(4)}
    for _ in range(len(joints)+1):
        for joint in joints:
            parent,child=joint.find('parent').attrib['link'],joint.find('child').attrib['link']
            if parent in frames and child not in frames: frames[child]=frames[parent]@transform(joint.find('origin'))
    total=0.;weighted=np.zeros(3)
    for link in links:
        inertial=link.find('inertial')
        if inertial is None:continue
        mass=float(inertial.find('mass').attrib['value'])
        if link.attrib['name'] not in frames:raise ValueError('Unresolved link transform')
        com=(frames[link.attrib['name']]@transform(inertial.find('origin')))[:3,3]
        total+=mass;weighted+=mass*com
    if total<=0:raise ValueError('Invalid model mass')
    return total,weighted/total


def curve_budgets(radius, com_height, track=0.18, lateral_offset=0., yaw_rate=3.6, gravity=9.81):
    if not all(math.isfinite(v) and v>0 for v in (radius,com_height,track,yaw_rate,gravity)):
        raise ValueError('Invalid geometric inputs')
    margin=track/2-abs(lateral_offset)
    if margin<=0:raise ValueError('COM outside support width')
    roll_accel=gravity*margin/com_height
    return {'radius_m':radius,'kinematic_yaw_limit_m_s':yaw_rate*radius,
            'ideal_rigid_rollover_limit_m_s':math.sqrt(roll_accel*radius),
            'friction_scenarios_m_s':{str(mu):math.sqrt(mu*gravity*radius) for mu in (.6,.8,1.0)},
            'warning':'Rollover/friction scenarios are idealized; no collision, slip or tracking guarantee.'}


def braking_budget(observation_range=2.5, braking=1.8, reaction=.25, margin=.3):
    if not all(math.isfinite(v) and v>0 for v in (observation_range,braking,reaction,margin)) or margin>=observation_range:
        raise ValueError('Invalid braking assumptions')
    return math.sqrt((braking*reaction)**2+2*braking*(observation_range-margin))-braking*reaction


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model',default=str(Path(__file__).resolve().parents[1]/'urdf/tianracer.xacro'))
    parser.add_argument('--yaw-rate',type=float,default=3.6)
    args=parser.parse_args()
    mass,com=model_com(args.model)
    report={'mass_kg':mass,'nominal_zero_steer_com_from_ground_m':com.tolist(),
            'gravity_assumption_m_s2':9.81,'track_assumption_m':.18,
            'turns':[curve_budgets(r,com[2],lateral_offset=com[1],yaw_rate=args.yaw_rate) for r in (.5,.75,1.,1.5)],
            'assumed_braking_budget_m_s':braking_budget(),
            'braking_assumptions':{'range_m':2.5,'deceleration_m_s2':1.8,'reaction_s':.25,'margin_m':.3},
            'under_120s_constant_pace_equivalent_m_s':1.8*142/120,
            'warning':'No safe maximum speed is established. The constant-pace estimate ignores corners and stops.'}
    print(json.dumps(report,indent=2,ensure_ascii=False))
