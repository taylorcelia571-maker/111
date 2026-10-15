#!/usr/bin/env python3
"""Inherit a completed local session; change one setting; never calls start.

Dry-run is default. --run starts the existing scene/referee/recorder workflow,
with manual prepare/status/start still required. No result is certified here.
"""
import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import yaml


PROFILES = {'baseline': None, 'algorithm': ('route_speed_refine', True),
            'accel_75': ('acceleration', 7.5),
            'lateral_101': ('lateral_accel', 10.1), 'lateral_102': ('lateral_accel', 10.2)}
FORWARD = ('lateral_accel', 'acceleration', 'braking', 'yaw_rate', 'startup_speed',
           'startup_acceleration', 'steering_feedback_gain', 'steering_preview_time',
           'heading_feedback_gain', 'cross_track_gain', 'line_search_level', 'predict_pose',
           'startup_connector_length', 'speed_recovery_hold', 'route_control_scoring',
           'runtime_path_cache', 'steering_preview_mode', 'route_joint_polish',
           'route_polish_strength', 'route_speed_refine', 'route_speed_iterations')


def read_object(path):
    data = json.loads(path.read_text())
    if not isinstance(data, dict):
        raise ValueError('Expected JSON object: ' + str(path))
    return data


def choose_session(root, selection):
    if selection not in ('best', 'latest'):
        session = Path(selection).expanduser().resolve()
    else:
        pointer = root / ('best_race_session.txt' if selection == 'best' else 'final_last_session.txt')
        if selection == 'best' and not pointer.exists() and (root / 'best70_session.txt').exists():
            pointer = root / 'best70_session.txt'
        if selection == 'best' and not pointer.exists():
            candidates = []
            for note in (root / 'final71').glob('*/referee_observation.json'):
                try:
                    item = read_object(note)
                    run = read_object(note.parent / 'run_report.json')
                    total = item.get('displayed_total_seconds')
                    if (item.get('valid_5_of_5') is True and item.get('any_incident') is False and
                            'last_fault' in run and run['last_fault'] is None and
                            isinstance(total, (int, float)) and not isinstance(total, bool) and
                            math.isfinite(total) and total > 0):
                        candidates.append((total, note.parent))
                except (OSError, ValueError):
                    continue
            if not candidates:
                raise ValueError('No saved best session. If the latest completed session is your valid 70s run, use --base latest; no file hunting required.')
            session = min(candidates, key=lambda item: item[0])[1].resolve()
        else:
            session = Path(pointer.read_text().strip()).expanduser().resolve()
    if not session.is_dir():
        raise ValueError('Session not found: ' + str(session))
    if (session / 'control.bag.active').exists():
        raise ValueError('Source recording is still active; stop and close the old launch first')
    return session


def inherited_parameters(session):
    run = read_object(session / 'run_report.json')
    if 'last_fault' not in run or run['last_fault'] is not None:
        raise ValueError('Source must explicitly record last_fault=null')
    if run.get('run_sequence', 1) != 1:
        raise ValueError('Source contains multiple starts; do not use it as an isolated baseline')
    parameters = yaml.safe_load((session / 'params.yaml').read_text())
    if not isinstance(parameters, dict):
        raise ValueError('Missing source parameter dictionary')
    for name, expected in {'route_margin': .34, 'response_delay': .25, 'max_error': .15,
                           'max_speed': 10., 'steering_rate': 2.}.items():
        actual = parameters.get(name)
        if isinstance(actual, bool) or not isinstance(actual, (int, float)) or actual != expected:
            raise ValueError('Source boundary differs: ' + name)
    if parameters.get('lateral_control') != 'race_feedback' or parameters.get('speed_planning') != 'delayed_braking':
        raise ValueError('This trial runner requires the recorded race_feedback/delayed_braking baseline')
    if not 0 < parameters.get('braking', 0) <= 3.7:
        raise ValueError('Source braking exceeds the measured-review gate')
    observation = None
    if (session / 'referee_observation.json').exists():
        observation = read_object(session / 'referee_observation.json')
        if observation.get('any_incident') is True or observation.get('valid_5_of_5') is False:
            raise ValueError('Source observation records an invalid run or incident')
    defaults = dict(runtime_path_cache=False, steering_preview_mode='legacy',
                    route_joint_polish=False, route_polish_strength=.5,
                    route_speed_refine=False, route_speed_iterations=6)
    result = {}
    for name in FORWARD:
        if name in parameters:
            result[name] = parameters[name]
        elif name in defaults:
            result[name] = defaults[name]
        else:
            raise ValueError('Source parameters incomplete: ' + name)
    config = run.get('configuration') or {}
    if (session / 'planned_course_report.json').exists():
        plan = read_object(session / 'planned_course_report.json')
        if run.get('reference_sha256') != plan.get('reference_sha256'):
            raise ValueError('Source route/report checksum mismatch')
    for name in FORWARD:
        value = config.get(name)
        if value is not None and not isinstance(value, dict) and name in parameters and value != parameters[name]:
            raise ValueError('Parameter/report mismatch: ' + name)
    return result, observation


def make_recipe(session, profile):
    original, observation = inherited_parameters(session)
    parameters = dict(original)
    change = PROFILES[profile]
    changed = {}
    if change:
        name, value = change
        old = parameters[name]
        if old == value:
            raise ValueError('Selected profile would not change anything: ' + name)
        if name == 'lateral_accel' and not (0 < value - old <= .201 and value <= 10.2):
            raise ValueError('Lateral trial must increase by at most 0.2 and stay <=10.2')
        if name == 'acceleration' and not (0 < value - old <= .201):
            raise ValueError('Acceleration trial must increase by at most 0.2')
        parameters[name] = value
        changed[name] = dict(before=old, after=value)
    if not 0 < parameters['lateral_accel'] <= 10.2:
        raise ValueError('Lateral budget outside this experiment envelope')
    return dict(source_session=str(session), source_commit=(session / 'commit.txt').read_text().strip(),
                source_official_observation=observation, source_official_file_verified=False,
                profile=profile, changed=changed, parameters=parameters,
                budget_trial=parameters['lateral_accel'] > 9.94,
                official_result_verified=False, official_under_60_verified=False)


def launch_arguments(recipe):
    def text(value):
        return str(value).lower() if isinstance(value, bool) else str(value)
    arguments = ['baseline', 'legacy_control:=false']
    arguments += [name + ':=' + text(value) for name, value in recipe['parameters'].items()]
    arguments += ['budget_trial:=' + text(recipe['budget_trial'])]
    return arguments


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', default='best', help='best, latest or an explicit completed session directory')
    parser.add_argument('--profile', choices=PROFILES, default='baseline')
    parser.add_argument('--log-root', type=Path, default=Path.home() / 'ican_race_logs')
    parser.add_argument('--run', action='store_true', help='Without this flag, only print the inherited recipe')
    args = parser.parse_args()
    try:
        root = args.log_root.expanduser().resolve()
        session = choose_session(root, args.base)
        recipe = make_recipe(session, args.profile)
        print(json.dumps(recipe, ensure_ascii=False, indent=2, allow_nan=False), flush=True)
        print('Original official result is not certified here. Manual prepare/status/start required.', flush=True)
        if not args.run:
            print('Dry-run only; no ROS command issued. Add --run for the selected single trial.')
            return
        folder = root / 'speed_trial_recipes'
        folder.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        path = folder / (stamp + '_' + args.profile + '.json')
        with path.open('x') as stream:
            json.dump(recipe, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write('\n')
        env = dict(os.environ, RACE_TRIAL_RECIPE_FILE=str(path))
        script = Path(__file__).resolve().with_name('run_final_speedup.sh')
        result = subprocess.run(['bash', str(script)] + launch_arguments(recipe), env=env)
        sys.exit(result.returncode)
    except KeyboardInterrupt:
        sys.exit(130)
    except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError) as error:
        parser.error(str(error))


if __name__ == '__main__':
    main()
