#!/usr/bin/env python3
# ファイルの役割: ROS通信を偽物へ置き換え、画面を表示せずQtのキー・ボタン・速度入力イベントを検証する。
"""Qt event tests against the actual manual panel without moving Gazebo."""
import os
os.environ['QT_QPA_PLATFORM'] = 'offscreen'
from pathlib import Path
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from PySide2 import QtCore, QtGui, QtWidgets, QtTest
from manual_panel import ManualPanel, configure_japanese_font

ROOT = Path(__file__).resolve().parents[1]


# 画面テストでサービス要求と応答を代用する。
class FakeClient:
    # このクラスで使う状態・通信先・画面部品を初期化する。
    def __init__(self):
        self.values = []

    # テスト用のサービスが常に接続済みとして見えるようにする。
    def service_is_ready(self):
        return True

    # テストで要求された値を保存し、成功済みの疑似応答を返す。
    def call_async(self, request):
        self.values.append(request.data)
        return SimpleNamespace(done=lambda: True, result=lambda: SimpleNamespace(success=True, message='OK'))


# 画面テスト用にROSノードの必要な状態と送信履歴を代用する。
class FakeNode:
    # このクラスで使う状態・通信先・画面部品を初期化する。
    def __init__(self):
        self.mode = 'manual'
        self.state = 'stopped'
        self.state_time = time.monotonic()
        self.actual_speed = 0.0
        self.nearest = 1.42
        self.mode_client = FakeClient()
        self.enable_client = FakeClient()
        self.sent = []
        self.confirmed_auto_speed = .45
        self.pending_speed = None
        self.speed_future = None
        self.speed_error = ''

    # テスト用に、要求された自動速度をそのまま反映済み値へ保存する。
    def set_auto_speed(self, value):
        self.confirmed_auto_speed = value

    # 通信を行わない偽物なので何もしない。画面から同じ名前で呼び出せるようにする。
    def poll_auto_speed(self):
        pass

    # テスト用に送信要求の速度を履歴へ保存する。実際のROS配信は行わない。
    def send(self, v=0., w=0.):
        self.sent.append((v, w))


app = QtWidgets.QApplication([])
configure_japanese_font(app)


# 関連する検証ケースをまとめ、失敗条件をassertで確認するテストクラス。
class PanelTests(unittest.TestCase):
    # テストごとに偽の通信ノードと操作画面を用意し、実ROS通信を差し替える。
    def setUp(self):
        self.node = FakeNode()
        self.panel = ManualPanel(self.node)
        self.panel.timer.stop()
        self.panel.isActiveWindow = lambda: True
        self.panel.show()
        app.processEvents()
        self.panel.setFocus()
        self.spin = patch('manual_panel.rclpy.spin_once', lambda *a, **k: None)
        self.spin.start()
        self.panel.tick()

    # テスト用画面を閉じ、差し替えた通信処理を元に戻す。
    def tearDown(self):
        self.panel.close()
        self.spin.stop()

    # 前進と左旋回の同時押し、および両方を離した後の停止を確認する。
    def test_hold_release_and_combination(self):
        QtTest.QTest.keyPress(self.panel, QtCore.Qt.Key_W)
        QtTest.QTest.keyPress(self.panel, QtCore.Qt.Key_A)
        self.panel.tick()
        self.assertEqual(self.node.sent[-1], (.4, .65))
        QtTest.QTest.keyRelease(self.panel, QtCore.Qt.Key_W)
        QtTest.QTest.keyRelease(self.panel, QtCore.Qt.Key_A)
        self.panel.tick()
        self.assertEqual(self.node.sent[-1], (0., 0.))

    # 後退ボタンを押す間の後退速度と、離した直後の停止を確認する。
    def test_reverse_button_and_release(self):
        b = self.panel.directions['backward']
        QtTest.QTest.mousePress(b, QtCore.Qt.LeftButton)
        self.panel.tick()
        self.assertEqual(self.node.sent[-1], (-.25, 0.))
        QtTest.QTest.mouseRelease(b, QtCore.Qt.LeftButton)
        self.panel.tick()
        self.assertEqual(self.node.sent[-1], (0., 0.))

    # パネルが非アクティブになったとき、保持入力が消えて停止することを確認する。
    def test_window_deactivation_stops(self):
        QtTest.QTest.keyPress(self.panel, QtCore.Qt.Key_W)
        self.panel.tick()
        self.panel.isActiveWindow = lambda: False
        self.panel.changeEvent(QtCore.QEvent(QtCore.QEvent.ActivationChange))
        self.assertFalse(self.panel.held_keys)
        self.assertEqual(self.node.sent[-1], (0., 0.))

    # Spaceが停止ロックを要求し、保持中の方向キーを消すことを確認する。
    def test_space_locks_and_clears(self):
        QtTest.QTest.keyPress(self.panel, QtCore.Qt.Key_W)
        QtTest.QTest.keyPress(self.panel, QtCore.Qt.Key_Space)
        self.assertEqual(self.node.enable_client.values, [False])
        self.assertFalse(self.panel.held_keys)

    # 手動・自動の切替要求と同時に、古い方向入力が消えることを確認する。
    def test_mode_switch_clears_keys(self):
        QtTest.QTest.keyPress(self.panel, QtCore.Qt.Key_W)
        self.panel.set_mode(False)
        self.assertEqual(self.node.mode_client.values, [False])
        self.assertFalse(self.panel.held_keys)

    # 自動モード中に手動方向キーを押しても手動走行を要求しないことを確認する。
    def test_automatic_mode_ignores_keys(self):
        self.node.mode = 'auto'
        self.panel.tick()
        QtTest.QTest.keyPress(self.panel, QtCore.Qt.Key_W)
        self.panel.tick()
        self.assertEqual(self.node.sent[-1], (0., 0.))

    # スライダーと数値の同期、プリセット、自動速度ゼロの設定を確認する。
    def test_speed_controls(self):
        self.panel.forward_speed.slider.setValue(123)
        self.assertEqual(self.panel.forward_speed.value(), 1.23)
        self.panel.reverse_speed.spin.setValue(.72)
        self.assertEqual(self.panel.reverse_speed.slider.value(), 72)
        self.panel.apply_preset('fast')
        self.assertEqual(self.panel.forward_speed.value(), 1.)
        self.assertEqual(self.panel.reverse_speed.value(), .5)
        self.assertEqual(self.node.confirmed_auto_speed, .9)
        self.panel.auto_speed.set_value(0.)
        self.assertEqual(self.node.confirmed_auto_speed, 0.)

    def test_mission_manual_overrides_late_ack(self):
        ui=self.panel.navigation.mission
        ui.pending={'id':'old','op':'start'}
        self.panel.set_mode(True)
        self.panel.navigation.data={'mission_reply':{'id':'old','ok':True}}
        ui.refresh()
        self.assertEqual(self.node.mode_client.values,[True])

    def test_mission_edit_and_saved_course_check(self):
        import tempfile
        ui=self.panel.navigation.mission
        self.panel.navigation.world=str(ROOT / 'worlds' / 'warehouse_course.sdf')
        self.panel.navigation.select(1.,2.);ui.add()
        self.panel.navigation.select(3.,4.);ui.add();ui.move(-1)
        self.assertEqual(ui.points()[0]['x'],3.)
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'route.json';ui.write_route(path)
            ui.table.setRowCount(0);ui.read_route(path)
            self.assertEqual(len(ui.points()),2)
            self.panel.navigation.world=str(ROOT / 'worlds' / 'oval_course.sdf')
            with self.assertRaises(ValueError):ui.read_route(path)
        ui.table.selectRow(0);ui.remove();self.assertEqual(ui.table.rowCount(),1)
        self.panel.navigation.reset();self.assertEqual(ui.table.rowCount(),0)

    def test_manual_overrides_delayed_goal_ack(self):
        import json
        from std_msgs.msg import String
        ui=self.panel.navigation
        ui.mode_after_goal=True;ui.pending_goal=[0.,0.]
        self.panel.set_mode(True)
        ui.receive(String(data=json.dumps({'goal':[0.,0.],'state':'navigating'})))
        self.assertEqual(self.node.mode_client.values,[True])

    def test_stop_overrides_pending_free_drive(self):
        from concurrent.futures import Future
        from types import SimpleNamespace
        ui=self.panel.navigation;future=Future()
        ui.pending.append((future,True))
        self.panel.stop()
        if not future.cancelled():future.set_result(SimpleNamespace(success=True))
        ui.refresh()
        self.assertEqual(self.node.mode_client.values,[])
        self.assertEqual(self.node.enable_client.values,[False])

    def test_course_reset_clears_goal_selection(self):
        ui=self.panel.navigation;ui.select(5.,3.);ui.pending_goal=[5.,3.]
        ui.reset()
        self.assertIsNone(ui.map.selection)
        self.assertIsNone(ui.pending_goal)

    # 操作パネルをPNGへ保存できることを確認し、文字・配置の目視確認用画像を残す。
    def test_render_panel(self):
        path = Path(__file__).resolve().parents[1]/'logs/manual_panel.png'
        self.assertTrue(self.panel.grab().save(str(path)))


if __name__ == '__main__':
    unittest.main()
