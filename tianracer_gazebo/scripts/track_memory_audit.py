#!/usr/bin/env python3
"""Offline same-budget memory/preview comparison; never launches a vehicle."""
import argparse
import json
from pathlib import Path
import statistics
import time
import numpy as np
from race60_audit import load_course, rear_path
from final_speedup_audit import BUDGETS, OPTIONS, simulate
from predictive_follower import PredictiveFollower


def benchmark(center):
    ordinary = PredictiveFollower(rear_path(center), **BUDGETS, **OPTIONS)
    cached = PredictiveFollower(rear_path(center), **BUDGETS, **dict(OPTIONS, runtime_path_cache=True))
    path = ordinary.path
    samples = []
    for s in np.linspace(0., 5 * path.length, 500):
        x, y = path.sample(s) + [.01, -.01]
        samples.append((x, y, s - .1, s + .6))
    query_ms = {'ordinary': [], 'cached': []}
    for repeat in range(6):
        # Alternate evaluation order to reduce cache/CPU warmup bias.
        methods = [('ordinary', path.project), ('cached', cached.track_memory.project)]
        if repeat % 2:
            methods.reverse()
        for name, project in methods:
            start = time.perf_counter()
            for sample in samples:
                project(*sample)
            query_ms[name].append(1000 * (time.perf_counter() - start) / len(samples))
    return dict(projection_median_ms={k: statistics.median(v) for k, v in query_ms.items()},
                scope='Projection only, offline wall timing; not full control cycle or lap-time gain',
                warmup_laps_required=0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--quick', action='store_true')
    parser.add_argument('--route-only', action='store_true')
    args = parser.parse_args()
    package = Path(__file__).resolve().parents[1]
    planner, anchors = load_course(package)
    planner.margin = .34
    planner.allowed = planner.clearance >= .34
    center = planner.build(anchors, optimize_time=True, refine_line=True, line_search_level=3,
                           response_delay=.25, speed_planning='delayed_braking', **BUDGETS)
    print('Frozen original Level-3 route; same user71 budgets; all times are model-only.', flush=True)
    cases = [('baseline', {}), ('memory_only', {'runtime_path_cache': True}),
             ('adaptive_012', {'runtime_path_cache': True, 'steering_preview_mode': 'curvature_adaptive'}),
             ('adaptive_010', {'runtime_path_cache': True, 'steering_preview_mode': 'curvature_adaptive',
                               'steering_preview_time': .10})]
    from joint_line_polish import polish_line
    route_budgets = dict(BUDGETS, response_delay=.25, speed_planning='delayed_braking')
    routes = {}
    route_reports = []
    for strength in (.5, 1.):
        name = 'joint_route_' + str(strength)
        routes[name], report = polish_line(planner, center, anchors, route_budgets, strength=strength)
        report['name'] = name
        route_reports.append(report)
        print(json.dumps(report), flush=True)
    if args.route_only:
        cases = [('baseline', {})]
    cases += [(name, {}) for name in routes]
    responses = [(.08, .02, 0.), (.15, .02, 0.), (.15, .04, .04),
                 (.25, .04, .04), (.15, .02, .12)]
    if args.quick:
        responses = responses[:2]
    rows = []
    for name, overrides in cases:
        for lag, pose_age, joint_age in responses:
            row = simulate(routes.get(name, center), planner, dict(OPTIONS, **overrides), lag, pose_age, joint_age)
            row['name'] = name
            rows.append(row)
            print(json.dumps({k: row.get(k) for k in ('name', 'hypothetical_actuator_lag', 'pose_age',
                'joint_transport_age', 'completed', 'total_seconds', 'peak_reference_error', 'fault')}), flush=True)
    output = args.output.expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    result = dict(official_result=False, official_under_60_verified=False, gazebo_tested=False,
                  no_lap_time_improvement_claimed=True, route_selection=planner.selection,
                  minimum_route_clearance=planner.segment_check(center.points),
                  joint_route_reports=route_reports,
                  benchmark=benchmark(center), rows=rows,
                  assumptions='Same fixed route/budgets. Bicycle plus hypothetical first-order speed/steering lag; '
                    'no tire dynamics, roll, AMCL jumps, laser/referee validation. '
                    'Faster model time with worse error/drop is not acceptance evidence.')
    output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print('Saved model-only comparison: ' + str(output), flush=True)


if __name__ == '__main__':
    main()
