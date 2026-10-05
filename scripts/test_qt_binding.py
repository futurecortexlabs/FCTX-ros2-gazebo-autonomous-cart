#!/usr/bin/env python3
"""Qt binding regression: map buffers and real subprocess completion without Gazebo."""
import gc
import os
os.environ['QT_QPA_PLATFORM'] = 'offscreen'
import sys
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from PySide2 import QtCore, QtWidgets
import shiboken2
from nav_msgs.msg import OccupancyGrid
from goal_panel import GoalPanel

app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


class Owner:
    def button(self, label, action=None):
        button = QtWidgets.QPushButton(label)
        if action is not None:
            button.clicked.connect(action)
        return button


class QtBindingTests(unittest.TestCase):
    def setUp(self):
        self.panel = GoalPanel(SimpleNamespace(), Owner())
        self.panel.timer.stop()
        self.panel.mission.timer.stop()

    def tearDown(self):
        # 失敗時も、このテストが作った子だけを回収してから画面を破棄する。
        process = self.panel.obstacle_process
        if process is not None and shiboken2.isValid(process):
            process.kill()
            process.waitForFinished(2000)
        self.panel.close()
        self.panel.deleteLater()
        QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)

    def test_map_image_retains_pixels_after_temporary_buffer_is_gone(self):
        # ROS地図は下から上、画面は上から下。奇数幅でも行末を誤読しない。
        self.panel.data = {'frame': 'map'}
        message = OccupancyGrid()
        message.info.width, message.info.height = 3, 2
        message.info.resolution = .5
        message.info.origin.position.x = -1.
        message.info.origin.position.y = -2.
        message.data = [-1, 0, 100, 49, 50, -1]
        self.panel.receive_grid(message)
        image = self.panel.map.map_image
        self.assertEqual(self.panel.map.map_rectangle, (-1., .5, -2., -1.))
        expected = [[210, 20, 100], [100, 210, 20]]
        # 元メッセージを書き換えてGC後に再描画してもQImageの所有コピーは有効。
        message.data = [100] * 6
        gc.collect()
        self.panel.map.show()
        app.processEvents()
        self.assertFalse(self.panel.map.grab().isNull())
        self.assertEqual([[image.pixelColor(x, y).red() for x in range(3)] for y in range(2)], expected)
        self.panel.receive_grid(message)
        self.assertEqual(self.panel.map.map_image.pixelColor(0, 0).red(), 20)
        self.assertEqual(image.pixelColor(0, 0).red(), 210)

    def test_real_process_callback_reads_output_and_can_run_again(self):
        # Gazebo命令だけを小さな子へ置換し、実際のfinishedシグナルと破棄を検証。
        native_process = QtCore.QProcess
        class ProbeProcess(native_process):
            def start(self, program, arguments):
                self.requested = (program, arguments)
                super().start(sys.executable, ['-c', "print('完了: Qt subprocess')"])

        self.panel.data = {'frame': 'world'}
        with patch('goal_panel.QtCore.QProcess', ProbeProcess):
            for remove in (False, True):
                self.panel.obstacle_action(remove)
                process = self.panel.obstacle_process
                arguments = process.requested[1]
                self.assertEqual('--remove-all' in arguments, remove)
                deadline = time.monotonic() + 5
                while self.panel.obstacle_process is not None and time.monotonic() < deadline:
                    app.processEvents()
                    time.sleep(.005)
                self.assertIsNone(self.panel.obstacle_process, 'finished callback did not release process')
                self.assertEqual(self.panel.obstacle_notice.text(), '完了: Qt subprocess')
                QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)
                self.assertFalse(shiboken2.isValid(process), 'deleteLater did not destroy Qt child')


if __name__ == '__main__':
    unittest.main()
