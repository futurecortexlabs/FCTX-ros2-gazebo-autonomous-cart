"""Inject incomplete verdicts and failed second-launch exits without starting Gazebo."""
from pathlib import Path
import json
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import validate_shutdown_timing as timing


class ShutdownVerdicts(unittest.TestCase):
    def test_partial_and_cleanup_incomplete_results_remain_false(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'result.json'
            first = dict(passed=True, cleanup_completed=True)
            timing.write_record(path, [first], 2)
            self.assertFalse(json.loads(path.read_text())['all_passed'])
            timing.write_record(path, [first], 2, completed=True)
            self.assertFalse(json.loads(path.read_text())['all_passed'])
            second = dict(passed=True, cleanup_completed=False)
            timing.write_record(path, [first, second], 2, completed=True)
            self.assertFalse(json.loads(path.read_text())['all_passed'])

    def test_failure_after_case_success_is_recorded_as_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'result.json'
            cases = [dict(passed=True, cleanup_completed=True)]
            timing.write_record(path, cases, 1)
            self.assertFalse(json.loads(path.read_text())['all_passed'])
            timing.write_record(path, cases, 1, completed=True, failure='Final ROS cleanup failed')
            data = json.loads(path.read_text())
            self.assertFalse(data['all_passed'])
            self.assertEqual(data['failure'], 'Final ROS cleanup failed')
            timing.write_record(path, cases, 1, completed=True)
            self.assertTrue(json.loads(path.read_text())['all_passed'])

    def test_second_launch_negative_exit_cannot_pass(self):
        with patch.object(timing, 'group_members', return_value=[]):
            timing.verify_launch(SimpleNamespace(poll=lambda: 0), 10)
            with self.assertRaisesRegex(AssertionError, 'Owned launch exit'):
                timing.verify_launch(SimpleNamespace(poll=lambda: -2), 20)

    def test_second_launch_residual_process_cannot_pass(self):
        with patch.object(timing, 'group_members', return_value=[21]):
            with self.assertRaisesRegex(AssertionError, 'Owned process group remains'):
                timing.verify_launch(SimpleNamespace(poll=lambda: 0), 20)

    def test_startup_cancel_still_requires_exit_and_empty_group(self):
        with patch.object(timing, 'group_members', return_value=[]):
            timing.verify_launch(SimpleNamespace(poll=lambda: -2), 10, require_zero=False)
            with self.assertRaisesRegex(AssertionError, 'still running'):
                timing.verify_launch(SimpleNamespace(poll=lambda: None), 10, require_zero=False)


if __name__ == '__main__':
    unittest.main()
