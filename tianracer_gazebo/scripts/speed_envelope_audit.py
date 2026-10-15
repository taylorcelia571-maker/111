#!/usr/bin/env python3
"""Read-only model audit: same-budget algorithm, then one budget change at a time."""
import argparse
import json
from pathlib import Path
from race60_audit import load_course, rear_path
from final_speedup_audit import BUDGETS, OPTIONS, simulate
from predictive_follower import PredictiveFollower
from joint_line_polish import polish_line
from speed_aware_line import refine_speed_line, sector_speed_report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--quick', action='store_true')
    args = parser.parse_args()
    package = Path(__file__).resolve().parents[1]
    planner, anchors = load_course(package)
    planner.margin = .34
    planner.allowed = planner.clearance >= .34
    profiles = [('base_proxy', {}, False), ('algorithm', {}, True),
                ('algorithm_accel75', {'acceleration': 7.5}, True),
                ('algorithm_lat101', {'lateral_accel': 10.1}, True),
                ('algorithm_lat102', {'lateral_accel': 10.2}, True)]
    responses = [(.08, .02, 0.), (.15, .02, 0.), (.15, .04, .04),
                 (.25, .04, .04), (.15, .02, .12)]
    if args.quick:
        responses = responses[:2]
    planning, rows = [], []
    for name, delta, enabled in profiles:
        budgets = dict(BUDGETS, **delta)
        center = planner.build(anchors, optimize_time=True, refine_line=True, line_search_level=3,
            response_delay=.25, speed_planning='delayed_braking', **budgets)
        unpolished = center
        motion = dict(budgets, response_delay=.25, speed_planning='delayed_braking')
        center, polish = polish_line(planner, center, anchors, motion, strength=.5)
        refinement = None
        if enabled:
            center, refinement = refine_speed_line(planner, center, anchors, motion, iterations=6, source_seed=unpolished)
        follower = PredictiveFollower(rear_path(center), **budgets, **OPTIONS)
        record = dict(name=name, budgets=budgets, joint_polish=polish, speed_refinement=refinement,
                      reference_sha256=follower.path.fingerprint(),
                      minimum_route_clearance=planner.segment_check(center.points),
                      model_lap_seconds=float(sum(follower.path.ds / follower.path.speeds)),
                      sectors=sector_speed_report(follower.path))
        planning.append(record)
        print(json.dumps(dict(name=name, model_lap_seconds=record['model_lap_seconds'],
            accepted=refinement['accepted'] if refinement else None)), flush=True)
        for lag, pose, joint in responses:
            row = simulate(center, planner, OPTIONS, lag, pose, joint, budgets=budgets)
            row['name'] = name
            rows.append(row)
            print(json.dumps({k: row.get(k) for k in ('name', 'hypothetical_actuator_lag', 'pose_age',
                'joint_transport_age', 'completed', 'total_seconds', 'peak_reference_error',
                'maximum_single_cycle_command_drop', 'fault')}), flush=True)
    result = dict(official_result=False, official_under_60_verified=False, gazebo_tested=False,
                  user_reported_best_seconds=70, actual_70_second_session_available=False,
                  baseline_is_model_proxy=True, planning=planning, rows=rows,
                  assumptions='Official-map geometry with hypothetical bicycle/first-order actuators. '
                    'Baseline proxy assumes joint polish strength .5; it is not the unprovided actual 70s session. '
                    'Each budget profile replans as the real launch does. No tires, roll, laser/referee validation. '
                    'Experimental budget bounds are not measured safe ranges.')
    output = args.output.expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print('Saved model-only audit: ' + str(output), flush=True)


if __name__ == '__main__':
    main()
