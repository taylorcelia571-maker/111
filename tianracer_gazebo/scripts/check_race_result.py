#!/usr/bin/env python3
"""Compare telemetry with a user-transcribed referee result; never edit results."""
import argparse
import json
import math
import statistics
from pathlib import Path


def positive(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError(name + ' must be a finite positive number')
    return float(value)


def analyze(plan, run, official_seconds, official_laps, target=60.0):
    seconds = positive(official_seconds, 'official_seconds')
    target = positive(target, 'target')
    if isinstance(official_laps, bool) or not isinstance(official_laps, int) or not 0 <= official_laps <= 5:
        raise ValueError('official_laps must be an integer between 0 and 5')
    problems = []
    geometry = plan.get('reference_sha256')
    if not geometry or geometry != run.get('reference_sha256'):
        problems.append('Reference identity missing or different: use reports from the same updated run')
    if run.get('controller') != plan.get('lateral_control'):
        problems.append('Controller differs between plan and run')
    limits = run.get('limits', {})
    for key in ('max_speed', 'lateral_accel', 'acceleration', 'braking', 'yaw_rate', 'steering_rate'):
        if key not in limits or key not in plan.get('limits', {}) or limits[key] != plan['limits'][key]:
            problems.append('Parameter missing or different between plan and run: ' + key)
    if 'response_delay' not in limits or limits['response_delay'] != plan.get('response_delay_seconds'):
        problems.append('Response delay missing or different between plan and run')
    observed = positive(run.get('observed_simulation_seconds'), 'observed_simulation_seconds')
    if run.get('last_fault'):
        problems.append('Controller recorded a protective stop')
    if 'last_fault' not in run:
        problems.append('Run report has no fault status')
    if official_laps != 5:
        problems.append('Official referee did not complete five valid laps')
    reasons = []
    for key, row in run.get('time_by_state', {}).items():
        if ':' in key:  # phase, road and lap bins overlap the reason bins
            continue
        duration = row.get('seconds', 0)
        if not isinstance(duration, (int, float)) or not math.isfinite(duration) or duration < 0:
            raise ValueError('Invalid state duration: ' + key)
        reasons.append(dict(reason=key, seconds=duration, fraction_of_controller_run=duration/observed,
                            mean_measured_speed=row.get('mean_measured_speed')))
    reasons.sort(key=lambda row: row['seconds'], reverse=True)
    loops = run.get('reference_loop_seconds', [])
    for value in loops:
        positive(value, 'reference_loop_seconds')
    if official_laps == 5 and (len(loops) < 4 or observed < seconds*.9):
        problems.append('Run report appears partial: stop the controller and copy its final report')
    steady = statistics.median(loops[1:]) if len(loops) > 1 else None
    return dict(
        evidence_source='User-transcribed referee time; check environment eligibility and binding in referee GUI',
        independently_verified_official_result=False,
        supplied_official_laps=official_laps, supplied_official_seconds=seconds,
        target_seconds=target, seconds_above_target=max(0, seconds-target),
        telemetry_consistent=not problems,
        supplied_result_meets_time_target=not problems and seconds < target,
        problems=problems, controller_run_seconds=observed,
        model_five_seconds=plan.get('model_five_seconds'),
        response_preview_proxy_five_seconds=plan.get('response_preview_proxy_five_seconds'),
        max_speed_is_binding=plan.get('speed_ceiling_binding'),
        peak_planned_speed=plan.get('peak_planned_speed'), top_speed_restrictions=reasons,
        reference_first_loop_seconds=loops[0] if loops else None,
        reference_steady_median_seconds=steady,
        reference_startup_excess_seconds=max(0, loops[0]-steady) if steady is not None else None,
        reference_times_are_not_official_lap_times=True,
        startup=run.get('startup', {}), last_fault=run.get('last_fault'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--official-seconds', type=float, required=True)
    parser.add_argument('--official-laps', type=int, required=True)
    parser.add_argument('--target', type=float, default=60.0)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    try:
        plan = json.loads(args.plan.expanduser().read_text())
        run = json.loads(args.run.expanduser().read_text())
        report = analyze(plan, run, args.official_seconds, args.official_laps, args.target)
        output = args.output.expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False)+'\n')
        print(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False))
    except (OSError, ValueError, TypeError, AttributeError) as error:
        parser.error(str(error))
    return 0 if report['supplied_result_meets_time_target'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
