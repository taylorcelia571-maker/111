#!/usr/bin/env python3
"""Read session telemetry/bag. Never commands ROS or certifies official results."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import statistics
import numpy as np


TOPICS = ['/clock', '/tf', '/tf_static', '/tianracer/cmd_vel',
          '/tianracer/ackermann_cmd_stamped', '/tianracer/odom',
          '/tianracer/joint_states', '/tianracer/scan', '/tianracer/amcl_pose',
          '/tianracer/imu', '/rosout_agg']


def summarize_cycles(rows):
    cycles = [r for r in rows if r.get('event') == 'cycle']
    if not cycles:
        return {'available': False}
    reasons = Counter()
    first_turn = []
    entered = recovered = None
    drop = 0.0
    drop_rate = 0.0
    for previous, row in zip(cycles, cycles[1:]):
        dt = row['simulation_time'] - previous['simulation_time']
        if dt <= 0:
            continue
        reasons[previous['control'].get('reason', 'unknown')] += dt
        difference = max(0., previous['output_speed'] - row['output_speed'])
        drop = max(drop, difference)
        drop_rate = max(drop_rate, difference / dt)
    for row in cycles:
        curve = abs(row['control'].get('reference_curvature', 0.))
        if entered is None and row['progress'] > 1 and curve >= .35:
            entered = row['elapsed']
        if entered is not None and recovered is None:
            first_turn.append(row)
            if curve < .25 and row['elapsed'] - entered >= .3:
                recovered = row['elapsed']
    elapsed = cycles[-1]['elapsed']
    curves = [r for r in cycles if abs(r['control'].get('reference_curvature', 0)) >= .35]
    return dict(available=True, samples=len(cycles), observed_elapsed=elapsed,
        reason_seconds=dict(reasons), reason_attribution='previous cycle final reason; excludes overlapping road/phase/lap bins',
        peak_reference_error=max(r['control'].get('tracking_error', 0.) for r in cycles),
        p95_reference_error=float(np.quantile([r['control'].get('tracking_error', 0.) for r in cycles], .95)),
        maximum_single_sample_command_drop=drop, maximum_command_drop_rate=drop_rate,
        command_drop_rate_is_not_measured_braking=True,
        peak_speed_excess=max(r['control'].get('speed_excess', 0.) for r in cycles),
        maximum_joint_age=max((r['control'].get('steering_feedback_age') or 0.) for r in cycles),
        first_turn=dict(entry_elapsed=entered,
            entry_speed=first_turn[0]['measured_speed'] if first_turn else None,
            minimum_speed=min(r['measured_speed'] for r in first_turn) if first_turn else None,
            exit_elapsed=recovered,
            duration=recovered-entered if recovered is not None else None,
            definition='reference curvature >=0.35/m after 1m; exit <0.25/m after >=0.3s; not an official gate'),
        curve_speed_median=statistics.median(r['measured_speed'] for r in curves) if curves else None)


def estimate_braking(commands, odometry, reaction=.25):
    """Fit measured straight-line deceleration after the configured allowance.

    Produces observations, not a certified minimum deceleration. Windows with
    turning/lateral motion, too little speed change or poor fit are rejected.
    """
    invalid = sum(not all(math.isfinite(r[k]) for k in ('t','v','omega','lateral')) for r in odometry)
    odometry = [r for r in odometry if all(math.isfinite(r[k]) for k in ('t','v','omega','lateral'))]
    commands = [r for r in commands if all(math.isfinite(r[k]) for k in ('t','v'))]
    if invalid:
        return {'available':False,'reason':'Nonfinite odometry in braking evidence',
                'invalid_samples':invalid,'permission_to_raise_braking':False}
    if len(commands) < 2 or len(odometry) < 5:
        return {'available': False, 'reason': 'Not enough command/odom data',
                'permission_to_raise_braking': False}
    odometry = sorted(odometry, key=lambda r: r['t'])
    clean = []
    rejected = 0
    last_event = -math.inf
    for old, command in zip(commands, commands[1:]):
        # Planned braking is usually gradual at 50Hz. Detect a cumulative drop
        # over 200ms, not only abrupt single-command drops that bias the evidence.
        recent=[r for r in commands if command['t']-.2 <= r['t'] < command['t']]
        approach=max([old['v']] + [r['v'] for r in recent])
        if approach - command['v'] < .3 or approach < 2 or command['t'] - last_event < 1.:
            continue
        last_event = command['t']
        low, high = command['t'] + reaction, command['t'] + reaction + .7
        rises = [r['t'] for r in commands if command['t'] < r['t'] <= high and r['v'] > command['v'] + .2]
        if rises:
            high = min(high, min(rises))
        samples = [r for r in odometry if low <= r['t'] <= high and r['v'] > .5]
        if (len(samples) < 5 or samples[-1]['t'] - samples[0]['t'] < .3 or
            max(abs(r['omega']) for r in samples) > .3 or max(abs(r['lateral']) for r in samples) > .15):
            rejected += 1
            continue
        t = np.array([r['t'] - low for r in samples]); v = np.array([r['v'] for r in samples])
        slope, intercept = np.polyfit(t, v, 1)
        residual = float(np.sum((v - (intercept + slope*t))**2))
        variation = float(np.sum((v-v.mean())**2))
        r2 = 1-residual/max(variation, 1e-12)
        if slope >= 0 or v[0]-v[-1] < .5 or r2 < .9:
            rejected += 1
            continue
        clean.append(dict(command_time=command['t'], window_start=low, window_end=high,
                          measured_deceleration=float(-slope), r_squared=r2, samples=len(samples)))
    return dict(available=bool(clean), windows=clean, rejected_events=rejected,
        minimum_observed_deceleration=min((r['measured_deceleration'] for r in clean), default=None),
        configured_response_allowance=reaction, allowance_is_not_measured_response=True,
        permission_to_raise_braking=False,
        note='Turning, slopes, slip, contact and hidden faults require separate review. Sparse windows do not prove braking budget safety.')


def read_bag(filename):
    import rosbag
    counts = Counter(); commands = []; odometry = []; observations = []; logs = []; joints=[]
    with rosbag.Bag(str(filename), 'r') as bag:
        for topic, msg, stamp in bag.read_messages(topics=TOPICS):
            counts[topic] += 1
            header = getattr(msg, 'header', None)
            t = header.stamp.to_sec() if header is not None and header.stamp.to_sec() > 0 else stamp.to_sec()
            if topic == '/tianracer/cmd_vel':
                commands.append(dict(t=t, v=msg.linear.x, steer=msg.angular.z))
            elif topic == '/tianracer/odom':
                v = msg.twist.twist
                row = dict(t=t, v=v.linear.x, lateral=v.linear.y, omega=v.angular.z,
                           position=[msg.pose.pose.position.x, msg.pose.pose.position.y])
                odometry.append(row); observations.append(dict(topic=topic, **row))
            elif topic == '/tianracer/ackermann_cmd_stamped':
                observations.append(dict(topic=topic,t=t,v=msg.drive.speed,steer=msg.drive.steering_angle))
            elif topic == '/tianracer/joint_states':
                positions=dict(zip(msg.name,msg.position))
                left=positions.get('left_steering_hinge_joint');right=positions.get('right_steering_hinge_joint')
                observations.append(dict(topic=topic,t=t,left=left,right=right))
                if left is not None and right is not None and all(math.isfinite(v) for v in (left,right)):
                    from wheel_kinematics import equivalent_steer
                    joints.append(dict(t=t,angle=equivalent_steer(left,right)))
            elif topic == '/tianracer/amcl_pose':
                p=msg.pose.pose; observations.append(dict(topic=topic,t=t,position=[p.position.x,p.position.y],quaternion=[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]))
            elif topic == '/tianracer/scan':
                ranges=np.asarray(msg.ranges)
                observations.append(dict(topic=topic,t=t,nan=int(np.isnan(ranges).sum()),negative_inf=int(np.isneginf(ranges).sum()),samples=len(ranges)))
            elif topic == '/rosout_agg':
                if msg.level >= 4:
                    logs.append(dict(t=t,node=msg.name,level=msg.level,text=msg.msg))
            elif topic in ('/tf','/tf_static'):
                observations.append(dict(topic=topic,t=t,transforms=[dict(
                    stamp=r.header.stamp.to_sec(),parent=r.header.frame_id,child=r.child_frame_id,
                    position=[r.transform.translation.x,r.transform.translation.y,r.transform.translation.z],
                    quaternion=[r.transform.rotation.x,r.transform.rotation.y,r.transform.rotation.z,r.transform.rotation.w]) for r in msg.transforms]))
            elif topic == '/clock':
                observations.append(dict(topic=topic,t=t,simulation_clock=msg.clock.to_sec()))
            elif topic == '/tianracer/imu':
                q=msg.orientation;norm=math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
                roll=pitch=None
                if math.isfinite(norm) and norm>1e-6:
                    x,y,z,w=q.x/norm,q.y/norm,q.z/norm,q.w/norm
                    roll=math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
                    pitch=math.asin(max(-1.,min(1.,2*(w*y-z*x))))
                observations.append(dict(topic=topic,t=t,roll=roll,pitch=pitch))
    return dict(topic_counts=dict(counts), braking=estimate_braking(commands,odometry),
                steering_response=estimate_steering_response(commands,joints),
                commands=commands, observations=observations, warnings_errors=logs)


def estimate_steering_response(commands, joints):
    """Fit an effective first-order response, including transport/time alignment.

    Delay and actuator constant are not independently identifiable from every lap.
    This is a descriptive fit, never evidence for relaxing a protection budget.
    """
    commands=[r for r in commands if all(math.isfinite(r[k]) for k in ('t','steer'))]
    joints=[r for r in joints if all(math.isfinite(r[k]) for k in ('t','angle'))]
    if len(commands)<50 or len(joints)<50:
        return dict(available=False,reason='Need continuous steering excitation',permission_to_change_safety=False)
    if any(b['t']<a['t'] for a,b in zip(commands,commands[1:])) or any(b['t']<a['t'] for a,b in zip(joints,joints[1:])):
        return dict(available=False,reason='Clock reset/mixed runs',permission_to_change_safety=False)
    times=np.array([r['t'] for r in commands]);angles=np.array([r['steer'] for r in commands])
    t=np.array([r['t'] for r in joints]);actual=np.array([r['angle'] for r in joints])
    dt=np.diff(t);valid=(dt>=.005)&(dt<=.10)
    measured_rate=np.diff(actual)/np.maximum(dt,1e-9)
    best=None
    for delay in np.arange(0,.151,.01):
        index=np.searchsorted(times,t[:-1]-delay,side='right')-1
        chosen=np.clip(index,0,len(times)-1)
        error=angles[chosen]-actual[:-1]
        mask=valid&(index>=0)&(np.abs(error)>=.01)&(np.abs(measured_rate)<5)
        if mask.sum()<50:continue
        x=error[mask];y=measured_rate[mask]
        gain=float(np.dot(x,y)/max(np.dot(x,x),1e-12))
        if gain<=0 or not .02<=1/gain<=.5:continue
        residual=float(np.mean((y-gain*x)**2))
        quality=1-residual/max(float(np.var(y)),1e-12)
        if best is None or residual<best['mean_squared_rate_residual']:
            best=dict(effective_time_constant_seconds=1/gain,effective_delay_seconds=float(delay),
                      mean_squared_rate_residual=residual,r_squared=quality,samples=int(mask.sum()))
    return dict(available=best is not None and best['r_squared']>=.4,fit=best,
        permission_to_change_safety=False,
        note='Effective joint response including transport/stamp alignment; low-quality or nonunique fit cannot certify servo delay.')


def analyze(session, bag=None):
    session=Path(session).expanduser()
    run=json.loads((session/'run_report.json').read_text())
    plan=json.loads((session/'planned_course_report.json').read_text())
    rows=[]; invalid_lines=0
    with (session/'cycles.jsonl').open() as stream:
        for line in stream:
            try: rows.append(json.loads(line))
            except ValueError: invalid_lines+=1
    cycles=summarize_cycles(rows)
    loops=run.get('reference_loop_seconds',[])
    result=dict(official_result_verified=False,official_under_60_verified=False,
        reference_matches=run.get('reference_sha256') == plan.get('reference_sha256'),
        configuration=run.get('configuration'),startup=run.get('startup'),last_fault=run.get('last_fault'),
        telemetry=run.get('telemetry'),invalid_jsonl_lines=invalid_lines,
        reference_first_loop=loops[0] if loops else None,
        reference_later_mean=statistics.mean(loops[1:5]) if len(loops)>=5 else None,
        reference_loops_are_not_official_laps=True,cycle_analysis=cycles,
        collision_count=None,rollover_count=None,
        unverified=['Official original microsecond result and lap validity','Collision/rollover visual evidence','Actual servo response','Fault-free measured braking capability'])
    telemetry=run.get('telemetry') or {}
    result['cycle_data_complete']=bool(telemetry and telemetry.get('dropped')==0 and
        telemetry.get('error') is None and telemetry.get('written')==telemetry.get('accepted') and not invalid_lines)
    bag_data=read_bag(Path(bag).expanduser()) if bag else None
    if bag_data:
        fault=(run.get('last_fault') or {}).get('simulation_time')
        if fault is None:
            cycle_rows=[r for r in rows if r.get('event')=='cycle']
            fault=cycle_rows[-1]['simulation_time'] if cycle_rows else None
        result['bag_topic_counts']=bag_data['topic_counts'];result['measured_braking']=bag_data['braking']
        result['effective_steering_response']=bag_data['steering_response']
        if fault is not None:
            result['last_five_seconds']=dict(end_time=fault,
                cmd_vel=[r for r in bag_data['commands'] if fault-5<=r['t']<=fault],
                observations=[r for r in bag_data['observations'] if fault-5<=r['t']<=fault],
                warnings_errors=[r for r in bag_data['warnings_errors'] if fault-5<=r['t']<=fault])
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session',type=Path,required=True)
    parser.add_argument('--bag',type=Path)
    args=parser.parse_args()
    try:
        result=analyze(args.session,args.bag)
        from cycle_telemetry import finite_json
        result=finite_json(result)
        output=args.session.expanduser()/'analysis.json'
        output.write_text(json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
        print(json.dumps({k:result[k] for k in ('reference_matches','last_fault','cycle_analysis')},indent=2,ensure_ascii=False))
        print('Saved '+str(output)+'; official result remains unverified.')
    except (OSError,ValueError,KeyError,ImportError) as error:
        parser.error(str(error))


if __name__=='__main__':main()
