import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import zipfile

spec = importlib.util.spec_from_file_location('collector', Path(__file__).resolve().parents[1]/
                                            'scripts/collect_race_diagnostics.py')
collector = importlib.util.module_from_spec(spec)
spec.loader.exec_module(collector)


class CollectorTests(unittest.TestCase):
    def test_compact_archive_uses_clip_and_preserves_original(self):
        with tempfile.TemporaryDirectory() as root:
            p = Path(root); logs = p/'logs'; logs.mkdir()
            original = logs/'A_control.bag'; original.write_bytes(b'large original evidence')
            (logs/'A_control.json').write_text('{"last_fault":{"simulation_time":117.8}}')
            def fake_clip(source, destination, reports):
                destination.write_bytes(b'fault window')
                return {'source': str(source), 'fault_time':117.8}
            with patch.object(collector,'command',return_value={'error':'ROS offline'}), \
                    patch.object(collector,'clip_bag',side_effect=fake_clip) as clipped:
                collector.collect(logs,p/'missing',p/'compact.zip',compact=True)
            self.assertEqual(clipped.call_count,1)
            self.assertEqual(original.read_bytes(),b'large original evidence')
            with zipfile.ZipFile(p/'compact.zip') as z:
                self.assertEqual(z.read('race_logs/A_control.bag'),b'fault window')
                self.assertEqual(json.loads(z.read('manifest.json'))['bag_windows'][0]['fault_time'],117.8)

    def test_clip_failure_keeps_report_and_records_missing_bag(self):
        with tempfile.TemporaryDirectory() as root:
            p=Path(root);logs=p/'logs';logs.mkdir()
            (logs/'A_control.bag').write_bytes(b'bag')
            (logs/'A_control.json').write_text('{}')
            with patch.object(collector,'command',return_value={'error':'ROS offline'}), \
                    patch.object(collector,'clip_bag',side_effect=ImportError('rosbag unavailable')):
                collector.collect(logs,p/'missing',p/'compact.zip',compact=True)
            with zipfile.ZipFile(p/'compact.zip') as z:
                self.assertIn('race_logs/A_control.json',z.namelist())
                self.assertNotIn('race_logs/A_control.bag',z.namelist())
                self.assertIn('rosbag unavailable',json.loads(z.read('manifest.json'))['skipped'][0]['reason'])

    def test_missing_ros_and_partial_bag_preserve_existing_evidence(self):
        with tempfile.TemporaryDirectory() as root:
            p = Path(root); logs = p/'logs'; logs.mkdir()
            (logs/'A_projection_v1.json').write_text('{"last_fault": "wall"}')
            (logs/'A_control.bag').write_bytes(b'bag evidence')
            (logs/'new.bag.active').write_bytes(b'partial')
            output = p/'collected.zip'
            with patch.object(collector, 'command', return_value={'error': 'ROS unavailable'}):
                collector.collect(logs, p/'missing_ros_logs', output)
            with zipfile.ZipFile(output) as z:
                self.assertEqual(z.read('race_logs/A_control.bag'), b'bag evidence')
                manifest = json.loads(z.read('manifest.json'))
                self.assertFalse(manifest['official_result_verified'])
                self.assertEqual(len(manifest['skipped']), 1)
            self.assertTrue((logs/'new.bag.active').exists())

    def test_does_not_overwrite_archive(self):
        with tempfile.TemporaryDirectory() as root:
            p = Path(root); output = p/'old.zip'; output.write_bytes(b'original')
            with patch.object(collector, 'command', return_value={'error': 'missing'}):
                with self.assertRaises(FileExistsError):
                    collector.collect(p/'missing', p/'missing_ros', output)
            self.assertEqual(output.read_bytes(), b'original')

    def test_current_session_logs_collected_without_vehicle_commands(self):
        with tempfile.TemporaryDirectory() as root:
            p = Path(root); session = p/'ros_logs'/'current-run'; session.mkdir(parents=True)
            (session/'tianracer-continuous_race-1.log').write_text('wall fault')
            output = p/'collected.zip'
            def fake(args):
                return {'stdout': 'current-run\n' if args == ['rosparam','get','/run_id'] else ''}
            with patch.object(collector, 'command', side_effect=fake) as calls:
                collector.collect(p/'missing', p/'ros_logs', output)
            with zipfile.ZipFile(output) as z:
                self.assertEqual(z.read('ros_logs/current-run/tianracer-continuous_race-1.log'), b'wall fault')
            for call in calls.call_args_list:
                self.assertNotIn('/tianracer/continuous_race/start', call.args[0])
                self.assertNotIn('/tianracer/continuous_race/stop', call.args[0])
                self.assertNotIn('set', call.args[0])


if __name__ == '__main__':
    unittest.main()
