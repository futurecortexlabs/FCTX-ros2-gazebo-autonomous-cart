"""強制終了と孤児プロセスを、成功の終了コードだけで見逃さない。"""
import unittest
from copy import deepcopy
from unittest.mock import Mock, patch
import validation_processes as processes
from validate_endurance import finalize_result as finalize_endurance
from validate_saved_routes import finalize_result as finalize_saved_routes


class OwnedStop(unittest.TestCase):
    def test_force_with_zero_exit_is_failure(self):
        launch = Mock(pid=100, returncode=0)
        launch.poll.return_value = 0
        with self.assertRaises(AssertionError):
            processes.verify_clean_stop(launch, [('SIGTERM', 1.)], {}, '')

    def test_detached_child_remains_but_reused_pid_does_not(self):
        child = Mock(pid=101)
        child.create_time.return_value = 10.
        child.status.return_value = processes.psutil.STATUS_RUNNING
        unrelated = Mock(pid=102)
        unrelated.create_time.return_value = 20.
        unrelated.status.return_value = processes.psutil.STATUS_RUNNING
        with patch.object(processes.psutil, 'process_iter', return_value=[child, unrelated]), \
             patch.object(processes.os, 'getpgid', return_value=500):
            self.assertEqual(processes.remaining_owned(100, {101: 10., 102: 1.}), [101])

    def test_orphan_child_with_zero_exit_is_failure(self):
        launch = Mock(pid=100, returncode=0)
        launch.poll.return_value = 0
        with patch.object(processes, 'remaining_owned', return_value=[101]):
            with self.assertRaises(AssertionError):
                processes.verify_clean_stop(launch, [('SIGINT', 1.)], {101: 10.}, '')

    def test_child_born_after_snapshot_in_owned_group_cannot_pass(self):
        late_child = Mock(pid=103)
        late_child.create_time.return_value = 15.
        late_child.status.return_value = processes.psutil.STATUS_RUNNING
        other_project = Mock(pid=104)
        other_project.create_time.return_value = 16.
        other_project.status.return_value = processes.psutil.STATUS_RUNNING
        launch = Mock(pid=100, returncode=0)
        launch.poll.return_value = 0
        # Neither process existed in the old snapshot. Only PGID100 is owned.
        with patch.object(processes.psutil, 'process_iter', return_value=[late_child, other_project]), \
             patch.object(processes.os, 'getpgid', side_effect=lambda pid: 100 if pid == 103 else 500):
            self.assertEqual(processes.remaining_owned(100, {}), [103])
            with self.assertRaisesRegex(AssertionError, 'Owned processes remain'):
                processes.verify_clean_stop(launch, [('SIGINT', 1.)], {}, '')


class FinalVerdicts(unittest.TestCase):
    def actions(self, events):
        return [('observer', lambda: events.append('observer')),
                ('node', lambda: events.append('node')),
                ('rclpy', lambda: events.append('rclpy'))]

    def test_endurance_interrupted_owned_cleanup_cannot_save_success(self):
        for interruption in (KeyboardInterrupt(), SystemExit(0)):
            with self.subTest(interruption=type(interruption).__name__):
                events, saved = [], []
                result = dict(passed=False, cleanup_completed=False)
                def interrupted():
                    events.append('owned')
                    raise interruption
                def save():
                    saved.append((events[:], deepcopy(result)))
                with self.assertRaises(RuntimeError):
                    finalize_endurance(result, True, interrupted, self.actions(events), save)
                self.assertEqual(saved[0][0], ['owned', 'observer', 'node', 'rclpy'])
                self.assertFalse(saved[0][1]['passed'])
                self.assertFalse(saved[0][1]['cleanup_completed'])
                self.assertIn(type(interruption).__name__, saved[0][1]['cleanup_failure'])

    def test_endurance_terminal_ros_failure_is_saved_after_remaining_cleanup(self):
        events, saved = [], []
        result = dict(passed=False)
        def broken():
            events.append('node')
            raise RuntimeError('injected ROS cleanup failure')
        actions = [('node', broken), ('rclpy', lambda: events.append('rclpy'))]
        with self.assertRaises(RuntimeError):
            finalize_endurance(result, True, lambda: events.append('owned'), actions,
                               lambda: saved.append((events[:], deepcopy(result))))
        self.assertEqual(saved[0][0], ['owned', 'node', 'rclpy'])
        self.assertTrue(saved[0][1]['cleanup_completed'])
        self.assertFalse(saved[0][1]['passed'])
        self.assertEqual(saved[0][1]['terminal_cleanup_errors'][0]['stage'], 'node')

    def test_endurance_success_is_saved_only_after_owned_and_ros_cleanup(self):
        events, saved = [], []
        result = dict(passed=False, cleanup_completed=False)
        finalize_endurance(result, True, lambda: events.append('owned'), self.actions(events),
                           lambda: saved.append((events[:], deepcopy(result))))
        self.assertEqual(saved[0][0], ['owned', 'observer', 'node', 'rclpy'])
        self.assertTrue(saved[0][1]['passed'])

    def test_saved_route_interrupt_returns_failure_and_runs_all_terminal_cleanup(self):
        for interruption in (KeyboardInterrupt(), SystemExit(0)):
            with self.subTest(interruption=type(interruption).__name__):
                events, saved = [], []
                result = dict(all_passed=True)
                def interrupted():
                    events.append('owned')
                    self.assertFalse(result['all_passed'])
                    raise interruption
                code = finalize_saved_routes(result, 0, interrupted, self.actions(events),
                                              lambda: saved.append((events[:], deepcopy(result))))
                self.assertEqual(code, 1)
                self.assertEqual(saved[0][0], ['owned', 'observer', 'node', 'rclpy'])
                self.assertFalse(saved[0][1]['all_passed'])
                self.assertFalse(saved[0][1]['final_cleanup_completed'])
                self.assertIn(type(interruption).__name__, saved[0][1]['cleanup_failure'])

    def test_saved_route_ros_failure_or_previous_trial_failure_cannot_turn_true(self):
        events, saved = [], []
        result = dict(all_passed=True)
        def broken():
            events.append('node')
            raise SystemExit(0)
        code = finalize_saved_routes(result, 0, lambda: events.append('owned'),
                                     [('node', broken), ('rclpy', lambda: events.append('rclpy'))],
                                     lambda: saved.append((events[:], deepcopy(result))))
        self.assertEqual(code, 1)
        self.assertEqual(saved[0][0], ['owned', 'node', 'rclpy'])
        self.assertFalse(saved[0][1]['all_passed'])
        failed = dict(all_passed=False, failure='delivery failed')
        self.assertEqual(finalize_saved_routes(failed, 1, lambda: None, [], lambda: None), 1)
        self.assertFalse(failed['all_passed'])


if __name__ == '__main__':
    unittest.main()
