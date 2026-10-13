#!/usr/bin/env python3
"""Read-only race evidence collection; never starts or commands the vehicle."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time
import zipfile


def command(args):
    try:
        result = subprocess.run(args, capture_output=True, text=True, timeout=8)
        return {'command': args, 'returncode': result.returncode,
                'stdout': result.stdout, 'stderr': result.stderr}
    except (OSError, subprocess.TimeoutExpired) as error:
        return {'command': args, 'error': str(error)}


def clip_bag(source, destination, reports):
    """Keep indexed fault context and static TF; original bag is read-only."""
    import rosbag
    import rospy
    matches = [p for p in reports if p.stem == source.stem]
    candidates = matches or reports
    with rosbag.Bag(str(source), 'r') as bag:
        begin, end = bag.get_start_time(), bag.get_end_time()
        fault = None
        matched = None
        for report in candidates:
            try:
                data = json.loads(report.read_text())
                value = (data.get('last_fault') or {}).get('simulation_time')
                if isinstance(value, (int, float)) and begin <= value <= end:
                    fault, matched = value, str(report)
                    break
            except (OSError, ValueError, AttributeError):
                continue
        # If no matching fault exists, keep the beginning, not the stationary tail.
        low = max(begin, fault-8) if fault is not None else begin
        high = min(end, fault+2) if fault is not None else min(end, begin+15)
        topics = ['/clock', '/rosout', '/rosout_agg', '/tf',
                  '/tianracer/cmd_vel', '/tianracer/odom', '/tianracer/joint_states',
                  '/tianracer/amcl_pose', '/tianracer/scan']
        count = 0
        with rosbag.Bag(str(destination), 'w') as target:
            for topic, msg, stamp in bag.read_messages(topics=['/tf_static']):
                target.write(topic, msg, stamp)
                count += 1
            for topic, msg, stamp in bag.read_messages(
                    topics=topics, start_time=rospy.Time.from_sec(low),
                    end_time=rospy.Time.from_sec(high)):
                target.write(topic, msg, stamp)
                count += 1
    return {'source': str(source), 'start': low, 'end': high,
            'fault_time': fault, 'report': matched, 'messages': count,
            'report_match': 'filename' if matches else 'timestamp_only',
            'note': 'Timestamp-only matches may refer to another run; check parameters.'}


def collect(log_dir, ros_log_dir, output, compact=False):
    log_dir, ros_log_dir, output = map(Path, (log_dir, ros_log_dir, output))
    output.parent.mkdir(parents=True, exist_ok=True)
    manifest = {'created_local': time.strftime('%Y-%m-%d %H:%M:%S'),
                'official_result_verified': False,
                'note': 'Files may describe different runs; compare timestamps and parameters.',
                'files': [], 'skipped': [], 'commands': [], 'bag_windows': []}
    for args in (['rosparam', 'get', '/run_id'],
                 ['rosparam', 'get', '/tianracer/continuous_race'],
                 ['rosservice', 'call', '/tianracer/continuous_race/status', '{}'],
                 ['rostopic', 'info', '/tianracer/cmd_vel'],
                 ['rostopic', 'list']):
        manifest['commands'].append(command(args))
    candidates = []
    if log_dir.is_dir():
        # Include recent reports and all existing bag files, without overwriting.
        reports = sorted(log_dir.glob('*.json'), key=lambda p: p.stat().st_mtime,
                         reverse=True)[:12]
        candidates.extend((p, 'race_logs/'+p.name) for p in reports)
        candidates.extend((p, 'race_logs/'+p.name) for p in log_dir.glob('*.csv'))
        bags = sorted(log_dir.glob('*.bag'), key=lambda p: p.stat().st_mtime, reverse=True)
        candidates.extend((p, 'race_logs/'+p.name) for p in (bags[:2] if compact else bags))
        for p in log_dir.glob('*.bag.active'):
            manifest['skipped'].append({'path': str(p), 'reason':
                'Recording is not finalized. Ctrl+C in the recording terminal, then collect again.'})
    sessions = []
    run_id = manifest['commands'][0].get('stdout', '').strip().strip('"\'')
    if run_id and '/' not in run_id and (ros_log_dir/run_id).is_dir():
        sessions.append(ros_log_dir/run_id)
    if (ros_log_dir/'latest').is_dir():
        sessions.append((ros_log_dir/'latest').resolve())
    if ros_log_dir.is_dir():
        recent = sorted((p for p in ros_log_dir.iterdir()
                         if p.is_dir() and not p.is_symlink()),
                        key=lambda p: p.stat().st_mtime, reverse=True)[:3]
        sessions.extend(recent)
    for session in dict.fromkeys(sessions):
        for p in session.glob('*'):
            if p.is_file() and any(k in p.name.lower() for k in
                                  ('continuous_race', 'amcl', 'referee', 'roslaunch', 'rosout')):
                candidates.append((p, 'ros_logs/'+session.name+'/'+p.name))
    with tempfile.TemporaryDirectory(prefix='race_fault_') as temporary, \
            zipfile.ZipFile(output, 'x', compression=zipfile.ZIP_DEFLATED,
                         compresslevel=6, allowZip64=True) as archive:
        for p, name in candidates:
            try:
                stat = p.stat()
                if stat.st_size > 512*1024*1024:
                    manifest['skipped'].append({'path': str(p), 'reason': 'File exceeds 512 MiB'})
                    continue
                if compact and p.suffix == '.bag':
                    try:
                        clipped = Path(temporary)/p.name
                        manifest['bag_windows'].append(clip_bag(p, clipped, reports))
                        archive.write(clipped, name)
                    except Exception as error:
                        manifest['skipped'].append({'path': str(p), 'reason':
                            'Could not clip bag: '+str(error)})
                        continue
                else:
                    archive.write(p, name)
                manifest['files'].append({'path': str(p), 'archive_path': name,
                                         'bytes': stat.st_size, 'mtime': stat.st_mtime})
            except OSError as error:
                manifest['skipped'].append({'path': str(p), 'reason': str(error)})
        archive.writestr('manifest.json', json.dumps(manifest, ensure_ascii=False, indent=2))
        archive.writestr('README.txt',
            'This archive collects existing evidence only. It does not start, stop or tune a car.\n'
            'Official lap results are not inferred from reference loops.\n'
            'Missing/unfinalized files are listed in manifest.json.\n')
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--logs', type=Path, default=Path.home()/'ican_race_logs')
    parser.add_argument('--ros-logs', type=Path, default=Path.home()/'.ros/log')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--compact', action='store_true', help='Only recent bags and fault windows')
    args = parser.parse_args()
    output = args.output or args.logs/('race_diagnostics_'+time.strftime('%Y%m%d_%H%M%S')+
                                     '_'+str(os.getpid())+'.zip')
    result = collect(args.logs, args.ros_logs, output, compact=args.compact)
    print('\n故障资料已打包：'+str(output.resolve()))
    print('大小：%.1f MiB；文件数：%d' % (output.stat().st_size/1024**2, len(result['files'])))
    if result['skipped']:
        print('注意：有未完成或未收集的文件，见压缩包 manifest.json。')
    if output.stat().st_size > 30*1024*1024:
        folder = output.with_suffix(''); folder.mkdir()
        with output.open('rb') as source:
            index = 1
            while True:
                chunk = source.read(20*1024*1024)
                if not chunk: break
                (folder/('part_%03d.bin' % index)).write_bytes(chunk)
                index += 1
        print('超过上传限制，已自动分成20 MiB小文件：'+str(folder))
        print('请上传该目录里的全部 part 文件；原压缩包保留。')
    else:
        print('请上传这个 ZIP；工具没有重新启动车辆，也没有修改参数。')


if __name__ == '__main__':
    main()
