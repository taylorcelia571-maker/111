#!/usr/bin/env python3
"""Reproducible map/route audit. Never starts ROS or produces official results."""
import argparse
import copy
import json
import math
from pathlib import Path
import numpy as np
import yaml
from loop_planner import LoopPlanner
from predictive_follower import PredictiveFollower
from race_line import ClosedPath, positive


def load_course(package):
    meta = yaml.safe_load((package / 'maps/tianracer_racetrack.yaml').read_text())
    with (package / 'maps' / meta['image']).open('rb') as stream:
        if stream.readline().strip() != b'P5':
            raise ValueError('Expected binary PGM map')
        line = stream.readline()
        while line.startswith(b'#'):
            line = stream.readline()
        width, height = map(int, line.split())
        if stream.readline().strip() != b'255':
            raise ValueError('Expected 8-bit PGM map')
        image = np.frombuffer(stream.read(), dtype=np.uint8).reshape(height, width)[::-1]
    # Conservative offline threshold: ambiguous gray cells remain blocked.
    planner = LoopPlanner(np.where(image >= 250, 0, 100), meta['resolution'], meta['origin'][:2])
    data = yaml.safe_load((package / 'scripts/waypoint_race/tianracer_racetrack_points.yaml').read_text())
    anchors = np.array([[v['pose']['position'][k] for k in ('x', 'y')] for v in data['waypoints']])
    return planner, anchors


def rear_path(center):
    tangent = np.roll(center.points, -1, axis=0) - np.roll(center.points, 1, axis=0)
    tangent /= np.linalg.norm(tangent, axis=1)[:, None]
    rear = center.points - .13 * tangent
    return ClosedPath(np.vstack([rear, rear[0]]), spacing=.04)


def simulate(center, planner, budgets, delay, plant_lag, laps=5,
             lateral_control='curvature_feedback'):
    """Hypothetical first-order actuators; excludes tires, AMCL and referee gates."""
    follower = PredictiveFollower(rear_path(center), response_delay=delay,
                                  lateral_control=lateral_control, fast_start=True, **budgets)
    x, y, yaw = .0081, -.1494, 1.5709
    speed = steer = peak_error = 0.0
    clearance = float('inf')
    follower.start(x, y, yaw)
    try:
        for tick in range(15000):
            follower.measured_steer = steer
            v, d = follower.step(x, y, yaw, speed, .02)
            speed += (v-speed) * .02 / plant_lag
            steer += (d-steer) * .02 / plant_lag
            x += speed * math.cos(yaw) * .02
            y += speed * math.sin(yaw) * .02
            yaw += speed * math.tan(steer) / .26 * .02
            peak_error = max(peak_error, follower.error)
            clearance = min(clearance, float(planner.clearance_at(
                [[x+.13*math.cos(yaw), y+.13*math.sin(yaw)]])[0]))
            if clearance < .23:
                raise ValueError('Model violates vehicle center clearance')
            if follower.progress-follower.origin >= laps*follower.path.length:
                return dict(completed=True, seconds=(tick+1)*.02,
                            peak_error=peak_error, minimum_center_clearance=clearance,
                            hypothetical_actuator_lag=plant_lag, official_result=False)
        raise ValueError('Model did not finish within 300 simulation seconds')
    except ValueError as error:
        return dict(completed=False, fault=str(error), seconds=(tick+1)*.02,
                    peak_error=peak_error, hypothetical_actuator_lag=plant_lag, official_result=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name, default in [('max-speed', 2.2), ('lateral-accel', 3.2), ('acceleration', 1.5),
                          ('braking', 2.0), ('yaw-rate', 2.5), ('steering-rate', 2.0),
                          ('response-delay', .25)]:
        parser.add_argument('--'+name, type=float, default=default)
    parser.add_argument('--route-margin', type=float, default=.34,
                        help='candidate center-route clearance, baseline remains .26m')
    parser.add_argument('--simulate', action='store_true')
    parser.add_argument('--lateral-control', choices=['pursuit', 'smooth_pursuit', 'curvature_feedback'],
                        default='curvature_feedback', help='match the controller actually used on the car')
    parser.add_argument('--lags', type=float, nargs='+', default=[.08, .15, .25])
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if any(not math.isfinite(lag) or not .02 <= lag <= 1 for lag in args.lags):
        parser.error('lags must be finite and between .02 and 1 second')
    budgets = {key: getattr(args, key) for key in
               ('max_speed', 'lateral_accel', 'acceleration', 'braking', 'yaw_rate', 'steering_rate')}
    planner, anchors = load_course(Path(__file__).resolve().parents[1])
    report = dict(official_result=False, official_five_laps_under_60_verified=False,
                  budgets=budgets, response_delay=args.response_delay,
                  candidate_route_margin=args.route_margin, lateral_control=args.lateral_control, routes={})
    for name, options in [('finish_baseline', dict(optimize_time=False)),
                          ('time_selected', dict(optimize_time=True)),
                          ('refined', dict(optimize_time=True, refine_line=True))]:
        margin=.26 if name=='finish_baseline' else args.route_margin
        planner.margin=positive('route_margin',margin)
        planner.allowed=planner.clearance>=margin
        center = planner.build(anchors, response_delay=args.response_delay, **budgets, **options)
        follower = PredictiveFollower(rear_path(center), response_delay=args.response_delay,
                                      lateral_control=args.lateral_control, **budgets)
        row = dict(selection=copy.deepcopy(planner.selection), plan=follower.plan_report(),
                   minimum_center_route_clearance=planner.segment_check(center.points),
                   route_margin=margin, wall_stop_clearance=.23)
        if args.simulate:
            row['hypothetical_tracking'] = [simulate(center, planner, budgets, args.response_delay, lag,
                                                     lateral_control=args.lateral_control)
                                             for lag in args.lags]
        report['routes'][name] = row
        print(name, 'envelope five=%.3fs; preview proxy five=%.3fs' % (
            row['plan']['model_five_seconds'], row['plan']['response_preview_proxy_five_seconds']), flush=True)
    args.output = args.output.expanduser()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print('Saved model audit (not official result):', args.output)


if __name__ == '__main__':
    main()
