"""起動・中止・正常終了・無応答時の監視と、勝手な再開を防ぐことを検証。"""
import signal
import time
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
from course_selector import CoursePanel, ROOT, COURSES


class StartupTests(unittest.TestCase):
    def panel(self, code=1, starting=True, source='simulation'):
        p = NS(
            shutdown_request=None, shutdown_request_since=None,
            shutdown_request_term_sent=False, shutdown_request_kill_sent=False,
            shutdown_request_kind='pause', shutdown_acknowledged=False,
            shutdown_pause_confirmed=False, shutdown_events=[],
            shutdown_slam_deactivation_started=False, shutdown_slam_deactivated=False,
            shutdown_startup_wait_since=None,
            stop_signal_sent=False, stop_signal_since=None,
            stop_term_sent=False, stop_kill_sent=False,
            process=NS(poll=lambda: code, pid=12345), starting=starting,
            stopping_since=None, startup_retries=0, startup_failure=None,
            closing=False, next_course=None, active_course=0,
            active_pose_source=source,
            pose_source=NS(findData=lambda _: 1, setCurrentIndex=Mock()),
            log=Mock(), load_button=Mock(), node=NS(state_time=0),
            course_status=Mock(), start_pending=Mock(), close=Mock(),
            setEnabled=Mock(),
            startup_since=time.monotonic(), stop_process=Mock(),
            startup_ready=lambda: False, signal_owned=Mock(),
            clear_input=Mock(), stop=Mock())
        p.poll_shutdown_request = lambda: CoursePanel.poll_shutdown_request(p)
        p.interrupt_owned_launch = lambda: CoursePanel.interrupt_owned_launch(p)
        p.prepare_launch_interrupt = lambda: CoursePanel.prepare_launch_interrupt(p)
        p.start_shutdown_request = lambda kind, command: CoursePanel.start_shutdown_request(p, kind, command)
        return p

    def request(self, code=None, output=b''):
        return NS(poll=lambda: code, returncode=code,
                  communicate=Mock(return_value=(output, None)),
                  terminate=Mock(), kill=Mock())

    def stopping_panel(self, elapsed, acknowledged=False):
        p = self.panel(code=None, starting=False)
        p.stopping_since = time.monotonic()-elapsed
        p.shutdown_acknowledged = acknowledged
        return p

    def test_only_one_startup_retry(self):
        p = self.panel()
        CoursePanel.poll_process(p)
        self.assertEqual(p.startup_retries, 1)
        self.assertEqual(p.next_course, 0)
        p = self.panel()
        p.startup_retries = 1
        CoursePanel.poll_process(p)
        self.assertIsNone(p.next_course)

    def test_runtime_failure_never_restarts(self):
        p = self.panel(starting=False)
        CoursePanel.poll_process(p)
        self.assertIsNone(p.next_course)

    def test_user_stop_never_restarts(self):
        p = self.panel()
        p.stopping_since = time.monotonic()
        CoursePanel.poll_process(p)
        self.assertIsNone(p.next_course)

    def test_pending_shutdown_request_blocks_next_start(self):
        p = self.panel()
        p.next_course = 2
        p.shutdown_request = self.request()
        p.shutdown_request_since = time.monotonic()
        CoursePanel.poll_process(p)
        p.start_pending.assert_not_called()

    def test_timeout_stops(self):
        p = self.panel(code=None)
        p.startup_since = time.monotonic()-61
        CoursePanel.poll_process(p)
        p.stop_process.assert_called_once()
        self.assertIsNotNone(p.startup_failure)

    def test_readiness_requires_fresh_pose_and_map(self):
        p = self.panel(source='slam')
        p.node.state_time = time.monotonic()
        p.node.state = 'stopped'
        p.navigation = NS(last_received=time.monotonic(),
            world=str(ROOT/'worlds'/COURSES[0][1]),
            data=dict(pose=[0, 0, 0], pose_source='slam'),
            map=NS(map_image=None))
        self.assertFalse(CoursePanel.startup_ready(p))
        p.navigation.map.map_image = object()
        self.assertTrue(CoursePanel.startup_ready(p))
        p.navigation.data['pose'] = None
        self.assertFalse(CoursePanel.startup_ready(p))

    def test_pending_stats_confirmation_does_not_interrupt_native(self):
        p = self.stopping_panel(6, acknowledged=True)
        p.shutdown_request = self.request()
        p.shutdown_request_since = time.monotonic()
        p.shutdown_request_kind = 'stats'
        CoursePanel.poll_process(p)
        p.signal_owned.assert_not_called()

    def test_failed_stats_cli_falls_back_to_a_single_launch_interrupt(self):
        p = self.stopping_panel(31, acknowledged=True)
        p.shutdown_request = self.request(-9)
        p.shutdown_request_kind = 'stats'
        CoursePanel.poll_process(p)
        p.signal_owned.assert_called_once_with(signal.SIGINT)
        self.assertIsNotNone(p.stop_signal_since)

    def test_unacknowledged_service_falls_back_to_launch_interrupt(self):
        p = self.stopping_panel(6)
        CoursePanel.poll_process(p)
        CoursePanel.poll_process(p)
        p.signal_owned.assert_called_once_with(signal.SIGINT)

    def test_pause_ack_starts_stats_confirmation_before_signal(self):
        p = self.panel(code=None)
        p.shutdown_request = self.request(0, b'data: true\n')
        p.start_shutdown_request = Mock()
        CoursePanel.poll_shutdown_request(p)
        self.assertTrue(p.shutdown_acknowledged)
        self.assertIsNone(p.shutdown_request)
        p.log.write.assert_called_once_with('data: true\n')
        self.assertEqual(p.start_shutdown_request.call_args.args[0], 'stats')
        p.signal_owned.assert_not_called()

    def test_unsuccessful_response_does_not_extend_native_cleanup_wait(self):
        for code, output in [(1, b'data: true\n'), (0, b'data: false\n')]:
            p = self.panel(code=None)
            p.shutdown_request = self.request(code, output)
            CoursePanel.poll_shutdown_request(p)
            self.assertFalse(p.shutdown_acknowledged)
            p.signal_owned.assert_called_once_with(signal.SIGINT)

    def test_stats_confirm_applied_pause_from_multiple_json_lines(self):
        p = self.panel(code=None)
        p.shutdown_request_kind = 'stats'
        p.shutdown_request = self.request(0, b'{"paused": false}\n{"paused": true}\n')
        CoursePanel.poll_shutdown_request(p)
        self.assertTrue(p.shutdown_pause_confirmed)
        self.assertEqual(p.shutdown_events[0][0], 'pause_applied')
        p.signal_owned.assert_called_once_with(signal.SIGINT)

    def test_missing_or_invalid_stats_never_claim_pause_applied(self):
        for output in (b'{"paused": false}\n', b'no response\n', b'null\n'):
            p = self.panel(code=None)
            p.shutdown_request_kind = 'stats'
            p.shutdown_request = self.request(0, output)
            CoursePanel.poll_shutdown_request(p)
            self.assertFalse(p.shutdown_pause_confirmed)
            p.signal_owned.assert_called_once_with(signal.SIGINT)

    def test_exited_launch_does_not_create_delayed_stats_or_signal(self):
        p = self.panel()
        p.shutdown_request = self.request(0, b'data: true\n')
        p.start_shutdown_request = Mock()
        CoursePanel.poll_shutdown_request(p)
        p.start_shutdown_request.assert_not_called()
        p.signal_owned.assert_not_called()

    def test_unavailable_cli_still_stops_owned_launch(self):
        p = self.panel(code=None)
        with patch('course_selector.subprocess.Popen', side_effect=FileNotFoundError):
            CoursePanel.start_shutdown_request(p, 'pause', ['missing-gz'])
        self.assertIsNone(p.shutdown_request)
        p.signal_owned.assert_called_once_with(signal.SIGINT)

    def test_pending_start_cannot_bypass_old_request_collection(self):
        p = self.panel()
        p.next_course = 1
        p.shutdown_request = self.request()
        with patch('course_selector.subprocess.Popen') as spawn:
            CoursePanel.start_pending(p)
        spawn.assert_not_called()
        self.assertEqual(p.next_course, 1)

    def test_hung_service_cli_is_terminated_then_killed_and_next_start_waits(self):
        p = self.panel()
        request = p.shutdown_request = self.request()
        p.shutdown_request_since = time.monotonic()-6
        CoursePanel.poll_process(p)
        CoursePanel.poll_process(p)
        request.terminate.assert_called_once()
        p.start_pending.assert_not_called()
        p.shutdown_request_since = time.monotonic()-9
        CoursePanel.poll_process(p)
        CoursePanel.poll_process(p)
        request.kill.assert_called_once()
        p.start_pending.assert_not_called()

    def test_early_cancel_targets_launch_without_sending_delayed_service(self):
        p = self.panel(code=None)
        with patch('course_selector.subprocess.Popen') as spawn:
            CoursePanel.stop_process(p)
        p.signal_owned.assert_called_once_with(signal.SIGINT)
        spawn.assert_not_called()
        self.assertIsNotNone(p.stop_signal_since)

    def test_existing_partition_is_used_for_owned_shutdown_request(self):
        p = self.panel(code=None, starting=False)
        with patch.dict('course_selector.os.environ', {'GZ_PARTITION': 'isolated_test'}):
            with patch('course_selector.subprocess.Popen') as spawn:
                CoursePanel.stop_process(p)
        self.assertEqual(spawn.call_args.kwargs['env']['GZ_PARTITION'], 'isolated_test')

    def test_signal_escalation_is_measured_from_interrupt_and_sent_once(self):
        p = self.stopping_panel(60, acknowledged=True)
        p.stop_signal_sent = True
        p.stop_signal_since = time.monotonic()-41
        CoursePanel.poll_process(p)
        CoursePanel.poll_process(p)
        p.signal_owned.assert_called_once_with(signal.SIGTERM, process_group=True)
        p.stop_signal_since = time.monotonic()-46
        CoursePanel.poll_process(p)
        CoursePanel.poll_process(p)
        self.assertEqual(p.signal_owned.call_count, 2)
        p.signal_owned.assert_called_with(signal.SIGKILL, process_group=True)

    def test_closing_waits_for_owned_launch_and_cancels_pending_course(self):
        p = self.panel(code=None)
        p.next_course = 2
        event = Mock()
        CoursePanel.closeEvent(p, event)
        self.assertTrue(p.closing)
        self.assertIsNone(p.next_course)
        p.stop_process.assert_called_once()
        event.ignore.assert_called_once()

    def test_exited_process_signal_race_does_not_abort_gui_cleanup(self):
        p = self.panel()
        with patch('course_selector.os.kill', side_effect=ProcessLookupError):
            CoursePanel.signal_owned(p, signal.SIGINT)
        self.assertEqual(p.shutdown_events, [])

    def test_slam_pause_confirmation_deactivates_before_interrupt(self):
        p = self.panel(code=None, starting=False, source='slam')
        p.shutdown_request_kind = 'stats'
        p.shutdown_request = self.request(0, b'{"paused": true}\n')
        pending = self.request()
        with patch('course_selector.subprocess.Popen', return_value=pending) as spawn:
            CoursePanel.poll_shutdown_request(p)
        self.assertEqual([event for event, _ in p.shutdown_events],
                         ['pause_applied', 'slam_deactivate_requested'])
        self.assertEqual(spawn.call_args.args[0], [
            'ros2', 'service', 'call', '/slam_toolbox/change_state',
            'lifecycle_msgs/srv/ChangeState', '{transition: {id: 4}}'])
        p.signal_owned.assert_not_called()

    def test_slam_deactivate_success_is_reaped_before_single_interrupt(self):
        p = self.panel(code=None, starting=False, source='slam')
        p.shutdown_slam_deactivation_started = True
        p.shutdown_request_kind = 'slam_deactivate'
        p.shutdown_request = self.request(0, b'response:\nlifecycle_msgs.srv.ChangeState_Response(success=True)\n')
        CoursePanel.poll_shutdown_request(p)
        CoursePanel.poll_shutdown_request(p)
        self.assertTrue(p.shutdown_slam_deactivated)
        self.assertIsNone(p.shutdown_request)
        self.assertEqual(p.shutdown_events[0][0], 'slam_deactivate_acknowledged')
        p.signal_owned.assert_called_once_with(signal.SIGINT)

    def test_slam_deactivate_rejection_or_failure_falls_back(self):
        for code, output, outcome in [(0, b'ChangeState_Response(success=False)\n', 'rejected'),
                                       (1, b'service unavailable\n', 'failed')]:
            p = self.panel(code=None, starting=False, source='slam')
            p.shutdown_slam_deactivation_started = True
            p.shutdown_request_kind = 'slam_deactivate'
            p.shutdown_request = self.request(code, output)
            CoursePanel.poll_shutdown_request(p)
            self.assertFalse(p.shutdown_slam_deactivated)
            self.assertEqual(p.shutdown_events[0][0], 'slam_deactivate_'+outcome)
            p.signal_owned.assert_called_once_with(signal.SIGINT)

    def test_slam_deactivate_timeout_is_bounded_and_collected_before_signal(self):
        p = self.panel(code=None, starting=True, source='slam')
        p.startup_ready = lambda: True
        request = self.request()
        with patch('course_selector.subprocess.Popen', return_value=request):
            CoursePanel.stop_process(p)
            CoursePanel.poll_process(p)
        self.assertEqual(p.shutdown_request_kind, 'slam_deactivate')
        p.signal_owned.assert_not_called()
        p.shutdown_request_since = time.monotonic()-6
        CoursePanel.poll_process(p)
        CoursePanel.poll_process(p)
        request.terminate.assert_called_once()
        p.shutdown_request_since = time.monotonic()-9
        CoursePanel.poll_process(p)
        request.kill.assert_called_once()
        p.signal_owned.assert_not_called()
        request.poll = lambda: -9
        request.returncode = -9
        CoursePanel.poll_process(p)
        self.assertIsNone(p.shutdown_request)
        self.assertIn('slam_deactivate_timed_out', [name for name, _ in p.shutdown_events])
        p.signal_owned.assert_called_once_with(signal.SIGINT)

    def test_slam_pause_or_stats_failure_still_attempts_deactivate(self):
        for kind, code, output in [('pause', 0, b'data: false\n'),
                                   ('stats', 1, b'unavailable\n')]:
            p = self.panel(code=None, starting=False, source='slam')
            p.shutdown_request_kind = kind
            p.shutdown_request = self.request(code, output)
            with patch('course_selector.subprocess.Popen', return_value=self.request()) as spawn:
                CoursePanel.poll_shutdown_request(p)
            self.assertEqual(p.shutdown_request_kind, 'slam_deactivate')
            self.assertEqual(spawn.call_args.args[0][0], 'ros2')
            p.signal_owned.assert_not_called()

    def test_slam_missing_cli_falls_back_without_recursive_retry(self):
        p = self.panel(code=None, starting=False, source='slam')
        with patch('course_selector.subprocess.Popen', side_effect=FileNotFoundError) as spawn:
            CoursePanel.start_shutdown_request(p, 'pause', ['missing-gz'])
        self.assertEqual(spawn.call_count, 2)
        self.assertEqual([event for event, _ in p.shutdown_events],
                         ['pause_unavailable', 'slam_deactivate_unavailable'])
        p.signal_owned.assert_called_once_with(signal.SIGINT)

    def test_repeated_slam_close_does_not_duplicate_deactivate(self):
        p = self.panel(code=None, starting=True, source='slam')
        p.stop_process = lambda: CoursePanel.stop_process(p)
        p.next_course = 2
        event = Mock()
        with patch('course_selector.subprocess.Popen', return_value=self.request()) as spawn:
            CoursePanel.closeEvent(p, event)
            original_deadline = p.shutdown_startup_wait_since
            CoursePanel.closeEvent(p, event)
            spawn.assert_not_called()
            self.assertEqual(p.shutdown_startup_wait_since, original_deadline)
            p.startup_ready = lambda: True
            CoursePanel.poll_process(p)
            CoursePanel.closeEvent(p, event)
        spawn.assert_called_once()
        self.assertTrue(p.closing)
        self.assertIsNone(p.next_course)
        p.signal_owned.assert_not_called()

    def test_slam_startup_cancel_waits_for_ready_and_never_restarts(self):
        p = self.panel(code=None, starting=True, source='slam')
        with patch('course_selector.subprocess.Popen', return_value=self.request()) as spawn:
            CoursePanel.stop_process(p)
            CoursePanel.poll_process(p)
            CoursePanel.poll_process(p)
            spawn.assert_not_called()
            p.signal_owned.assert_not_called()
            p.start_pending.assert_not_called()
            self.assertEqual(p.startup_retries, 0)
            p.startup_ready = lambda: True
            CoursePanel.poll_process(p)
            CoursePanel.poll_process(p)
        spawn.assert_called_once()
        self.assertIsNone(p.shutdown_startup_wait_since)
        self.assertEqual([event for event, _ in p.shutdown_events],
                         ['slam_startup_wait', 'slam_startup_ready', 'slam_deactivate_requested'])
        p.signal_owned.assert_not_called()

    def test_slam_startup_cancel_timeout_attempts_deactivate_once(self):
        p = self.panel(code=None, starting=True, source='slam')
        CoursePanel.stop_process(p)
        p.shutdown_startup_wait_since = time.monotonic()-21
        with patch('course_selector.subprocess.Popen', return_value=self.request()) as spawn:
            CoursePanel.poll_process(p)
            CoursePanel.poll_process(p)
        spawn.assert_called_once()
        self.assertIsNone(p.shutdown_startup_wait_since)
        self.assertIn('slam_startup_wait_timed_out', [event for event, _ in p.shutdown_events])
        p.signal_owned.assert_not_called()

    def test_slam_startup_wait_blocks_pending_course_and_old_launch_exit_clears_wait(self):
        p = self.panel(code=None, starting=True, source='slam')
        p.next_course = 2
        CoursePanel.stop_process(p)
        with patch('course_selector.subprocess.Popen') as spawn:
            CoursePanel.start_pending(p)
        spawn.assert_not_called()
        self.assertEqual(p.next_course, 2)
        p.process.poll = lambda: 0
        CoursePanel.poll_process(p)
        self.assertIsNone(p.shutdown_startup_wait_since)
        self.assertIsNone(p.process)
        self.assertEqual(p.startup_retries, 0)
        p.start_pending.assert_called_once()

    def test_slam_owned_mode_not_pending_combo_controls_deactivate(self):
        p = self.panel(code=None, starting=False, source='slam')
        p.pose_source.currentData = lambda: 'localization'
        with patch('course_selector.subprocess.Popen', return_value=self.request()) as spawn:
            CoursePanel.prepare_launch_interrupt(p)
        self.assertEqual(spawn.call_args.args[0][3], '/slam_toolbox/change_state')

    def test_late_slam_response_is_collected_before_next_course(self):
        p = self.panel(source='slam')
        p.next_course = 1
        request = p.shutdown_request = self.request()
        p.shutdown_request_kind = 'slam_deactivate'
        p.shutdown_request_since = time.monotonic()
        CoursePanel.poll_process(p)
        p.start_pending.assert_not_called()
        request.poll = lambda: 0
        request.returncode = 0
        request.communicate.return_value = (b'ChangeState_Response(success=True)\n', None)
        CoursePanel.poll_process(p)
        self.assertIsNone(p.shutdown_request)
        p.start_pending.assert_called_once()
        p.signal_owned.assert_not_called()

    def test_stopping_controls_reenable_only_after_old_launch_and_request(self):
        p = self.panel(code=None, starting=True, source='slam')
        p.next_course = 1
        CoursePanel.stop_process(p)
        p.setEnabled.assert_called_once_with(False)
        p.process.poll = lambda: 0
        p.shutdown_request_kind = 'slam_deactivate'
        request = p.shutdown_request = self.request()
        p.shutdown_request_since = time.monotonic()
        CoursePanel.poll_process(p)
        p.setEnabled.assert_called_once_with(False)
        order = []
        p.start_pending = lambda: order.append('start_pending')
        p.setEnabled.side_effect = lambda enabled: order.append(('enabled', enabled))
        request.poll = lambda: 0
        request.returncode = 0
        CoursePanel.poll_process(p)
        self.assertEqual(order, ['start_pending', ('enabled', True)])

    def test_closing_controls_stay_disabled_after_cleanup(self):
        p = self.panel(code=None, starting=True, source='slam')
        p.closing = True
        CoursePanel.stop_process(p)
        p.process.poll = lambda: 0
        CoursePanel.poll_process(p)
        p.setEnabled.assert_called_once_with(False)
        p.close.assert_called_once()


class ShutdownInputGuiTests(unittest.TestCase):
    """実Qtイベントで、停止中の新しい運転・自動・解除・地図操作を遮断する。"""

    def setUp(self):
        from PyQt5 import QtWidgets
        from test_manual_panel import FakeNode, app
        self.app = app
        self.node = FakeNode()
        self.client = NS(service_is_ready=lambda: True,
                         call_async=Mock(return_value=NS(done=lambda: False)))
        self.node.create_client = Mock(return_value=self.client)
        self.node.create_publisher = Mock(side_effect=lambda *_: Mock())
        self.node.create_subscription = Mock()
        self.panel = CoursePanel(self.node, simulation_gui=False)
        self.panel.timer.stop()
        self.panel.manager.stop()
        self.panel.navigation.timer.stop()
        self.panel.navigation.mission.timer.stop()
        self.panel.isActiveWindow = lambda: True
        self.panel.show()
        self.app.processEvents()
        self.panel.setFocus()
        self.panel.active_course = 0
        self.panel.active_pose_source = 'slam'
        self.panel.process = NS(poll=lambda: None, pid=12345)
        self.panel.starting = True
        self.panel.startup_ready = lambda: False
        self.panel.log = Mock()
        self.spin = patch('manual_panel.rclpy.spin_once')
        self.spin_mock = self.spin.start()
        self.panel.tick()

    def tearDown(self):
        from PyQt5 import QtCore
        self.panel.process = None
        self.panel.shutdown_request = None
        self.panel.close()
        self.panel.deleteLater()
        QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)
        self.spin.stop()

    def test_new_clicks_and_key_focus_cannot_restart_during_slam_wait(self):
        from PyQt5 import QtCore, QtWidgets, QtTest
        p = self.panel
        QtTest.QTest.keyPress(p, QtCore.Qt.Key_W)
        p.tick()
        self.assertEqual(self.node.sent[-1], (.4, 0.))
        # 起動途中は停止ロックのサービスがまだ無い場合もある。
        # backendのロックに頼らず、画面だけで新しい運転要求を遮断する。
        self.node.enable_client.service_is_ready = lambda: False
        p.stop_process()
        self.assertFalse(p.isEnabled())
        self.assertFalse(p.held_keys or p.held_buttons)
        self.assertEqual(self.node.sent[-1], (0., 0.))
        enabled_calls = list(self.node.enable_client.values)
        mode_calls = list(self.node.mode_client.values)
        resume = next(button for button in p.findChildren(QtWidgets.QPushButton)
                      if button.text() == '停止ロックを解除')
        p.navigation.last_received = time.monotonic()
        p.navigation.start.setEnabled(True)
        for button in [p.directions['forward'], p.auto, p.manual, resume, p.navigation.start]:
            self.assertFalse(button.isEnabled())
            QtTest.QTest.mouseClick(button, QtCore.Qt.LeftButton)
        p.setFocus()
        self.assertIsNot(QtWidgets.QApplication.focusWidget(), p)
        QtTest.QTest.keyPress(p, QtCore.Qt.Key_W)
        p.tick()
        self.assertEqual(self.node.sent[-1], (0., 0.))
        self.assertFalse(p.held_keys or p.held_buttons)
        self.assertEqual(self.node.enable_client.values, enabled_calls)
        self.assertEqual(self.node.mode_client.values, mode_calls)
        self.client.call_async.assert_not_called()
        p.navigation.pub.publish.assert_not_called()

    def test_disabled_panel_timers_still_receive_and_monitor_shutdown(self):
        from PyQt5 import QtTest
        p = self.panel
        p.timer.start(5)
        p.manager.start(5)
        p.stop_process()
        before = self.spin_mock.call_count
        QtTest.QTest.qWait(35)
        self.assertTrue(p.timer.isActive() and p.manager.isActive())
        self.assertGreater(self.spin_mock.call_count, before)
        self.assertIsNotNone(p.shutdown_startup_wait_since)
        self.assertTrue(self.node.sent)
        self.assertTrue(all(command == (0., 0.) for command in self.node.sent))

    def test_normal_startup_keeps_controls_and_navigation_enabled(self):
        from PyQt5 import QtCore, QtTest
        self.assertTrue(self.panel.isEnabled())
        QtTest.QTest.mousePress(self.panel.directions['forward'], QtCore.Qt.LeftButton)
        self.panel.tick()
        self.assertEqual(self.node.sent[-1], (.4, 0.))
        QtTest.QTest.mouseRelease(self.panel.directions['forward'], QtCore.Qt.LeftButton)


if __name__ == '__main__':
    unittest.main()
