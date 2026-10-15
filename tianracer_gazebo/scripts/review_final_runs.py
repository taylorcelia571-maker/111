#!/usr/bin/env python3
"""Post-run review only: no ROS publishers, service calls or parameter writes.

Referee values entered here are user transcriptions, never verified official
JSON. Controller/reference timing must not be promoted to an official result.
"""
import argparse
import csv
from datetime import datetime, timezone
from decimal import Decimal, InvalidOperation
import io
import json
import math
from pathlib import Path
import statistics
import subprocess
import sys
import zipfile


def read_json(path, issues):
    try:
        def reject(value):
            raise ValueError('Nonfinite JSON: ' + value)
        def finite_float(value):
            number = float(value)
            return number if math.isfinite(number) else reject(value)
        data = json.loads(path.read_text(encoding='utf-8'), parse_constant=reject, parse_float=finite_float)
        if not isinstance(data, dict):
            raise ValueError('Expected a JSON object')
        return data
    except (OSError, ValueError) as error:
        issues.append('{}: {}'.format(path.name, error))
        return None


def parse_seconds(text):
    """Accept decimal seconds or m:ss.ffffff without manufacturing precision."""
    text = text.strip()
    try:
        parts = text.split(':')
        if len(parts) == 1:
            seconds = Decimal(parts[0])
        elif len(parts) == 2 and parts[0].isdigit():
            tail = Decimal(parts[1])
            if not tail.is_finite() or not 0 <= tail < 60:
                raise ValueError('冒号后秒数应在 0 到 60 之间')
            seconds = Decimal(parts[0]) * 60 + tail
        else:
            raise ValueError('使用 72.125 或 1:12.125 格式')
        if not seconds.is_finite() or seconds <= 0:
            raise ValueError('时间必须为有限正数')
        value = float(seconds)
        if not math.isfinite(value) or value <= 0:
            raise ValueError('时间超出可表示范围')
        return value
    except InvalidOperation:
        raise ValueError('使用 72.125 或 1:12.125 格式')


def optional_bool(text):
    value = text.strip().lower()
    if value in ('y', 'yes', '是'):
        return True
    if value in ('n', 'no', '否'):
        return False
    if value in ('', 'u', 'unknown', '不确定'):
        return None
    raise ValueError('请输入 y、n 或 u')


def record_observation(session, ask=input):
    output = session / 'referee_observation.json'
    if output.exists():
        print('保留已有人工记录：' + str(output))
        return
    total_text = ask('裁判界面本轮总时间（如 1:12.125，未完赛也按界面录入）：').strip()
    total = parse_seconds(total_text)
    lap_text = ask('5 个单圈秒数，用空格分隔；看不到时直接回车：').strip()
    laps = [parse_seconds(item) for item in lap_text.replace(',', ' ').replace('，', ' ').split()]
    if laps and len(laps) != 5:
        raise ValueError('单圈时间应填 5 个，或全部留空')
    valid = optional_bool(ask('裁判是否确认本轮有效 5/5？[y/n/u]：'))
    incident = optional_bool(ask('本轮是否出现碰撞、翻车、保护停车、卡住或紧急制动任一项？[y/n/u]：'))
    data = dict(source='user_transcription_of_referee_ui',
                recorded_at=datetime.now(timezone.utc).isoformat(),
                displayed_total_text=total_text, displayed_total_seconds=total,
                displayed_lap_seconds=laps or None, valid_5_of_5=valid,
                any_incident=incident, referee_file_verified=False,
                original_time_microseconds=None, official_under_60_verified=False)
    with output.open('x', encoding='utf-8') as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print('已保存人工抄录；未认证官方原始结果：' + str(output))


def finite_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def lap_split(values):
    if not isinstance(values, list) or len(values) < 5 or not all(finite_number(v) and v > 0 for v in values[:5]):
        return None, None, None
    first = values[0]
    later = statistics.mean(values[1:5])
    return first, later, first - later


def summarize_session(session):
    issues = []
    run = read_json(session / 'run_report.json', issues)
    analysis = read_json(session / 'analysis.json', issues) if (session / 'analysis.json').exists() else {}
    note = read_json(session / 'referee_observation.json', issues) if (session / 'referee_observation.json').exists() else {}
    run, analysis, note = run or {}, analysis or {}, note or {}
    cycles = analysis.get('cycle_analysis') or {}
    config = run.get('configuration') or {}
    first, later, extra = lap_split(note.get('displayed_lap_seconds'))
    ref_first, ref_later, ref_extra = lap_split(run.get('reference_loop_seconds'))
    fault_recorded = 'last_fault' in run
    fault = run.get('last_fault')
    if not fault_recorded:
        issues.append('last_fault 未记录；不能当作 null')
    if run.get('run_sequence', 1) != 1:
        issues.append('同一节点多次 start，需人工拆分')
    if analysis.get('reference_matches') is False:
        issues.append('路径校验和不匹配')
    if analysis.get('cycle_data_complete') is False:
        issues.append('逐周期记录不完整')
    total = note.get('displayed_total_seconds')
    if total is not None and (not finite_number(total) or total <= 0):
        issues.append('人工总时间无效')
        total = None
    lap_times = note.get('displayed_lap_seconds')
    if total is not None and isinstance(lap_times, list) and len(lap_times) == 5 and first is not None:
        if abs(sum(lap_times) - total) > .02:
            issues.append('人工单圈之和与总时间不一致')
    active = (session / 'control.bag.active').exists()
    if active:
        issues.append('bag 尚未正常结束')
    if fault is not None or note.get('any_incident') is True or note.get('valid_5_of_5') is False:
        verdict = '失败/不纳入有效最快'
    elif (total is not None and note.get('valid_5_of_5') is True and
          note.get('any_incident') is False and fault_recorded and not issues):
        verdict = '用户记录有效；原始官方结果未核验'
    else:
        verdict = '数据不足/待核对'
    return dict(session=str(session), verdict=verdict, displayed_total_seconds=total,
                official_first_lap=first, official_later_four_mean=later,
                official_first_lap_extra=extra,
                reference_first_loop=ref_first, reference_later_four_mean=ref_later,
                reference_first_loop_extra=ref_extra,
                peak_reference_error=cycles.get('peak_reference_error', run.get('peak_reference_tracking_error')),
                maximum_command_drop=cycles.get('maximum_single_sample_command_drop'),
                maximum_joint_age=cycles.get('maximum_joint_age'),
                first_turn=cycles.get('first_turn'),
                final_reason_seconds=cycles.get('reason_seconds'),
                time_by_state_overlapping=run.get('time_by_state'),
                startup=run.get('startup'), cycle_wall_timing=run.get('cycle_wall_timing'),
                fault_recorded=fault_recorded, last_fault=fault, configuration=config,
                cycle_data_complete=analysis.get('cycle_data_complete'),
                issues=issues, official_under_60_verified=False)


def display(value):
    return '{:.3f}'.format(value) if finite_number(value) else '—'


def write_summary(root, sessions):
    rows = [summarize_session(path) for path in sessions]
    target = root.parent / 'final71_summary'
    payload = dict(official_under_60_verified=False, user_reported_historical_best_seconds=71,
                   note='人工抄录不等于已核验官方 JSON；参考圈不等于官方圈；reason/road/phase 统计有重叠，不相加。',
                   sessions=rows)
    target.with_suffix('.json').write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    columns = ['session', 'verdict', 'displayed_total_seconds', 'official_first_lap',
               'official_later_four_mean', 'official_first_lap_extra',
               'reference_first_loop', 'reference_later_four_mean', 'peak_reference_error',
               'maximum_command_drop', 'maximum_joint_age', 'last_fault', 'issues']
    stream = io.StringIO()
    writer = csv.DictWriter(stream, fieldnames=columns, extrasaction='ignore')
    writer.writeheader()
    writer.writerows(rows)
    target.with_suffix('.csv').write_text(stream.getvalue(), encoding='utf-8-sig')
    lines = ['# 五圈实验汇总', '', payload['note'], '',
             '历史最快 71 秒来自用户报告。总时间列只使用人工抄录的裁判界面值；无记录时留空。', '',
             '| 目录 | 裁判总秒数（抄录） | 判定 | 首圈 | 后四圈均值 | 首圈多用 | 最大参考偏差/m |',
             '| --- | ---: | --- | ---: | ---: | ---: | ---: |']
    for row in rows:
        lines.append('| {} | {} | {} | {} | {} | {} | {} |'.format(
            Path(row['session']).name.replace('|', '/'), display(row['displayed_total_seconds']), row['verdict'],
            display(row['official_first_lap']), display(row['official_later_four_mean']),
            display(row['official_first_lap_extra']), display(row['peak_reference_error'])))
    lines += ['', '配置、启动释放、首弯、指令降速、关节年龄、周期耗时、限速原因和数据缺项见同名 JSON。',
              '这些输出不能证明 official_result=true 或原始用时小于 60000000 微秒。', '']
    text = '\n'.join(lines)
    target.with_suffix('.md').write_text(text, encoding='utf-8')
    print(text)
    print('汇总文件：' + str(target) + '.{md,csv,json}')


def make_bundle(session, root):
    # Deliberate allowlist: do not include credentials, arbitrary directories or
    # the potentially multi-GB raw bag. It remains untouched in its run folder.
    names = ['commit.txt', 'command.txt', 'params.yaml', 'status_ready.txt',
             'trial_recipe.json', 'budget_trial.txt', 'library_versions.json',
             'status_prestart.txt', 'start_response.txt', 'stop_response.txt',
             'status_final.txt', 'run_report.json', 'planned_course_report.json',
             'planned_course.csv', 'C.log', 'record.log', 'analysis.json',
             'referee_observation.json', 'cycles.jsonl']
    destination = root.parent / (session.name + '_review.zip')
    included, missing, too_large = [], [], []
    with zipfile.ZipFile(destination, 'w', zipfile.ZIP_DEFLATED) as archive:
        for name in names:
            path = session / name
            if not path.is_file():
                missing.append(name)
            elif path.stat().st_size > 64 * 1024 * 1024:
                too_large.append(name)
            else:
                archive.write(str(path), arcname=session.name + '/' + name)
                included.append(name)
        archive.writestr(session.name + '/bundle_manifest.json', json.dumps(dict(
            included=included, missing=missing, over_64MiB=too_large,
            raw_bag_included=False, raw_bag_path=str(session / 'control.bag'),
            official_result_verified=False), ensure_ascii=False, indent=2))
    print('诊断包（不含原始 bag）：' + str(destination))
    if too_large:
        print('超过 64MiB 未打包：' + ', '.join(too_large))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path.home() / 'ican_race_logs' / 'final71')
    parser.add_argument('--record-latest', action='store_true')
    parser.add_argument('--analyze-latest', action='store_true')
    parser.add_argument('--bundle-latest', action='store_true')
    args = parser.parse_args()
    root = args.root.expanduser().resolve()
    try:
        if not root.is_dir():
            raise ValueError('未找到记录目录：' + str(root))
        if args.record_latest or args.analyze_latest or args.bundle_latest:
            pointer = root.parent / 'final_last_session.txt'
            latest = Path(pointer.read_text(encoding='utf-8').strip()).expanduser().resolve()
            if latest.parent != root or not latest.is_dir():
                raise ValueError('最新目录指针不属于本次 root：' + str(latest))
            if (latest / 'control.bag.active').exists():
                raise ValueError('本轮录制尚未正常结束。先 stop，再在驾驶终端 Ctrl+C；不要重发 start。')
            if args.record_latest:
                record_observation(latest)
            if args.analyze_latest:
                command = [sys.executable, str(Path(__file__).with_name('analyze_final_race.py')),
                           '--session', str(latest)]
                bag = latest / 'control.bag'
                if bag.exists():
                    command += ['--bag', str(bag)]
                else:
                    print('未找到已结束的 bag；仅分析逐周期记录，执行器/故障话题诊断不足。', file=sys.stderr)
                completed = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                if completed.returncode:
                    print('本轮分析失败，保留原始文件：\n' + completed.stderr, file=sys.stderr)
                else:
                    print('逐周期/可用 bag 分析完成：' + str(latest / 'analysis.json'))
            if args.bundle_latest:
                make_bundle(latest, root)
        sessions = sorted(path for path in root.iterdir() if path.is_dir() and
                          any((path / name).exists() for name in ('command.txt', 'run_report.json', 'C.log')))
        if not sessions:
            raise ValueError('未找到自动记录的实验。不会把模型结果当作官方成绩。')
        write_summary(root, sessions)
    except (OSError, ValueError, EOFError) as error:
        parser.error(str(error))


if __name__ == '__main__':
    main()
