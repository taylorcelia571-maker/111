import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import zipfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from review_final_runs import (make_bundle, optional_bool, parse_seconds,
                               read_json, record_observation, summarize_session)


class ReviewTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name) / 'final71'
        self.session = self.root / 'example_baseline'
        self.session.mkdir(parents=True)

    def put(self, name, value):
        (self.session / name).write_text(json.dumps(value), encoding='utf-8')

    def record(self, total='1:12.125', valid='y', incident='n', laps=''):
        answers = iter([total, laps, valid, incident])
        record_observation(self.session, ask=lambda _: next(answers))

    def test_time_formats_and_nonfinite_rejection(self):
        self.assertEqual(parse_seconds('1:12.125'), 72.125)
        self.assertEqual(parse_seconds('72.125'), 72.125)
        for value in ('NaN', 'Infinity', '-1', '0', '1:60', 'abc', '1:NaN', '1e999', '1e-999'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                parse_seconds(value)

    def test_unknown_answers_are_not_zero_incidents(self):
        self.assertIsNone(optional_bool(''))
        self.assertIsNone(optional_bool('u'))
        self.put('run_report.json', {'last_fault': None})
        self.record(incident='u')
        result = summarize_session(self.session)
        self.assertEqual(result['verdict'], '数据不足/待核对')

    def test_reference_time_never_becomes_official_time(self):
        self.put('run_report.json', {'last_fault': None, 'reference_loop_seconds': [10] * 5})
        result = summarize_session(self.session)
        self.assertIsNone(result['displayed_total_seconds'])
        self.assertFalse(result['official_under_60_verified'])

    def test_manual_under_sixty_is_not_certified(self):
        self.put('run_report.json', {'last_fault': None})
        self.record(total='59.500')
        result = summarize_session(self.session)
        self.assertFalse(result['official_under_60_verified'])
        note = json.loads((self.session / 'referee_observation.json').read_text())
        self.assertFalse(note['referee_file_verified'])
        self.assertIsNone(note['original_time_microseconds'])

    def test_crash_overrides_a_fast_displayed_time(self):
        self.put('run_report.json', {'last_fault': None})
        self.record(total='69.000', incident='y')
        self.assertEqual(summarize_session(self.session)['verdict'], '失败/不纳入有效最快')

    def test_missing_fault_field_not_assumed_null(self):
        self.put('run_report.json', {'observed_simulation_seconds': 70})
        self.record(total='70')
        result = summarize_session(self.session)
        self.assertFalse(result['fault_recorded'])
        self.assertEqual(result['verdict'], '数据不足/待核对')

    def test_lap_sum_inconsistency_and_restarts_flagged(self):
        self.put('run_report.json', {'last_fault': None, 'run_sequence': 2})
        self.record(total='70', laps='15 15 15 15 15')
        result = summarize_session(self.session)
        self.assertEqual(len(result['issues']), 2)
        self.assertAlmostEqual(result['official_first_lap_extra'], 0)
        self.assertEqual(result['verdict'], '数据不足/待核对')

    def test_existing_note_is_not_overwritten(self):
        self.record(total='72.125')
        record_observation(self.session, ask=lambda _: self.fail('Should not prompt'))
        note = json.loads((self.session / 'referee_observation.json').read_text())
        self.assertEqual(note['displayed_total_seconds'], 72.125)

    def test_protective_stop_overrides_manual_valid_claim(self):
        self.put('run_report.json', {'last_fault': {'reason': 'Tracking error exceeds safe corridor'}})
        self.record(total='70')
        self.assertEqual(summarize_session(self.session)['verdict'], '失败/不纳入有效最快')

    def test_cli_refuses_active_recording(self):
        (self.root.parent / 'final_last_session.txt').write_text(str(self.session))
        (self.session / 'control.bag.active').write_bytes(b'active')
        script = Path(__file__).resolve().parents[1] / 'scripts' / 'review_final_runs.py'
        result = subprocess.run([sys.executable, str(script), '--root', str(self.root), '--record-latest'],
                                input='', text=True, capture_output=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('不要重发 start', result.stderr)
        self.assertFalse((self.session / 'referee_observation.json').exists())

    def test_cli_writes_real_summary_artifacts(self):
        self.put('run_report.json', {'last_fault': None, 'reference_loop_seconds': [15, 14, 14, 14, 14]})
        (self.root.parent / 'final_last_session.txt').write_text(str(self.session))
        script = Path(__file__).resolve().parents[1] / 'scripts' / 'review_final_runs.py'
        result = subprocess.run([sys.executable, str(script), '--root', str(self.root),
                                 '--record-latest', '--bundle-latest'],
                                input='1:12.125\n16.125 14 14 14 14\ny\nn\n',
                                text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        summary = json.loads((self.root.parent / 'final71_summary.json').read_text())
        self.assertEqual(summary['sessions'][0]['displayed_total_seconds'], 72.125)
        self.assertEqual(summary['sessions'][0]['official_first_lap_extra'], 2.125)
        self.assertFalse(summary['official_under_60_verified'])
        self.assertTrue((self.root.parent / 'final71_summary.csv').exists())
        self.assertTrue((self.root.parent / 'final71_summary.md').exists())

    def test_nan_and_overflow_json_are_reported(self):
        for value in ('NaN', '1e999'):
            path = self.session / 'run_report.json'
            path.write_text('{"value":' + value + '}', encoding='utf-8')
            issues = []
            self.assertIsNone(read_json(path, issues))
            self.assertTrue(issues)

    def test_bundle_keeps_bag_private_and_untouched(self):
        self.put('run_report.json', {'last_fault': None})
        (self.session / 'control.bag').write_bytes(b'RAWBAG')
        (self.session / 'unrelated_secret.txt').write_text('not evidence')
        make_bundle(self.session, self.root)
        bundle = self.root.parent / 'example_baseline_review.zip'
        with zipfile.ZipFile(bundle) as archive:
            self.assertNotIn('example_baseline/control.bag', archive.namelist())
            self.assertNotIn('example_baseline/unrelated_secret.txt', archive.namelist())
            self.assertIn('example_baseline/run_report.json', archive.namelist())
        self.assertEqual((self.session / 'control.bag').read_bytes(), b'RAWBAG')


if __name__ == '__main__':
    unittest.main()
