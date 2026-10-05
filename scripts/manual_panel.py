#!/usr/bin/env python3
# ファイルの役割: Qtの操作画面とROS通信。押している間だけ手動指令を送り、自動速度の設定や停止要求も扱う。
"""ROS2 manual/automatic control panel. Held keys/buttons only; focus loss stops."""
import math
import os
from pathlib import Path
import sys
import signal
import time

os.environ.setdefault('ROS_DOMAIN_ID', '42')
os.environ.setdefault('QT_QPA_PLATFORM', 'xcb')
os.environ.setdefault('FONTCONFIG_FILE', str(Path(__file__).resolve().parents[1]/'fonts/fonts.conf'))
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool
from rclpy.qos import qos_profile_sensor_data
from PySide2 import QtCore, QtGui, QtWidgets
from speed_control import SpeedControl
from goal_panel import GoalPanel
from speed_settings import MAX_FORWARD_SPEED, MAX_REVERSE_SPEED, MAX_TURN_SPEED, MAX_AUTO_SPEED, PRESETS

KEYS = {
    QtCore.Qt.Key_W: 'forward', QtCore.Qt.Key_Up: 'forward',
    QtCore.Qt.Key_S: 'backward', QtCore.Qt.Key_Down: 'backward',
    QtCore.Qt.Key_A: 'left', QtCore.Qt.Key_Left: 'left',
    QtCore.Qt.Key_D: 'right', QtCore.Qt.Key_Right: 'right',
}
STATE_NAMES = {
    'driving': '走行中', 'slowing': '障害物に接近・減速中', 'stopped': '停止中',
    'user_stop': '停止ロック中', 'manual_input_timeout': '手動入力待ち・停止',
    'obstacle_stop': '障害物に接近・停止', 'invalid_scan_stop': '測定異常・停止',
    'waiting_for_fresh_data': 'センサー通信待ち・停止',
    'wall_contact_stop': '壁・障害物接触・再起動が必要', 'mode_switch_stop': '切替中・停止',
}



# 任意のMeiryoを登録し、未導入時はシステムのNoto CJKへ切り替える。
def configure_japanese_font(app):
    """Register regular and bold fonts, then explicitly apply the Japanese family."""
    folder = Path(__file__).resolve().parents[1]/'fonts'
    families = set()
    for name in ('meiryo.ttc', 'meiryob.ttc'):
        path = folder/name
        if not path.is_file():
            path = Path('/mnt/c/Windows/Fonts')/name
        if path.is_file():
            font_id = QtGui.QFontDatabase.addApplicationFont(str(path))
            families.update(QtGui.QFontDatabase.applicationFontFamilies(font_id))
    # 公開リポジトリはWindowsの字体を同梱せず、Ubuntuの日本語字体も利用する。
    families.update(QtGui.QFontDatabase().families())
    family = next((name for name in ('Meiryo', 'Noto Sans CJK JP') if name in families), app.font().family())
    font = QtGui.QFont(family, 10)
    font.setStyleStrategy(QtGui.QFont.PreferAntialias)
    app.setFont(font)
    return family


# 保持中の方向入力を並進速度と旋回速度へ変換する。反対方向の同時入力は相殺する。
def velocity(actions, speed, reverse_speed=.25, turn_speed=.65):
    direction = int('forward' in actions)-int('backward' in actions)
    v = speed if direction > 0 else (-reverse_speed if direction < 0 else 0.)
    w = turn_speed * (int('left' in actions)-int('right' in actions))
    return v, w


# 操作画面のROS通信を担当するノード。
class PanelNode(Node):
    # このクラスで使う状態・通信先・画面部品を初期化する。
    def __init__(self):
        super().__init__('loop_manual_panel')
        self.pub = self.create_publisher(Twist, '/loop/cmd_manual', 1)
        self.mode_client = self.create_client(SetBool, '/loop/set_manual')
        self.enable_client = self.create_client(SetBool, '/loop/set_enabled')
        self.speed_client = AsyncParameterClient(self, 'loop_autonomous_driver')
        self.confirmed_auto_speed = None
        self.pending_speed = None
        self.speed_future = None
        self.speed_request = None
        self.speed_error = ''
        self.last_speed_query = 0.0
        self.mode = ''
        self.state = ''
        self.state_time = 0.0
        self.actual_speed = 0.0
        self.nearest = math.inf
        self.create_subscription(String, '/loop/control_mode', lambda m: setattr(self, 'mode', m.data), 1)
        self.create_subscription(String, '/loop/safety_status', self.status, 1)
        self.create_subscription(Odometry, '/loop/ground_truth', self.odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/loop/scan', self.scan, qos_profile_sensor_data)

    # 画面で選ばれた自動速度を送信待ちにし、以前の通信エラー表示を消す。
    def set_auto_speed(self, value):
        self.pending_speed = float(value)
        self.speed_error = ''

    # 非同期パラメーター通信の完了を確認し、変更要求または定期的な設定値の読み出しを進める。
    def poll_auto_speed(self):
        if self.speed_future is not None and self.speed_future.done():
            try:
                result = self.speed_future.result()
                kind, value = self.speed_request
                if kind == 'get':
                    self.confirmed_auto_speed = result.values[0].double_value
                elif result.results and all(r.successful for r in result.results):
                    self.confirmed_auto_speed = value
                else:
                    self.speed_error = '自動速度を変更できませんでした'
            except Exception:
                self.speed_error = '自動速度の通信を確認してください'
            self.speed_future = None
        if self.speed_future is None and self.speed_client.services_are_ready():
            if self.pending_speed is not None:
                value, self.pending_speed = self.pending_speed, None
                self.speed_request = ('set', value)
                self.speed_future = self.speed_client.set_parameters([Parameter('max_speed', value=value)])
            elif self.confirmed_auto_speed is None or time.monotonic()-self.last_speed_query > 2.0:
                self.speed_request = ('get', None)
                self.speed_future = self.speed_client.get_parameters(['max_speed'])
                self.last_speed_query = time.monotonic()

    # 安全状態と受信実時間を保存する。画面側の接続切れ判定にも利用する。
    def status(self, msg):
        self.state, self.state_time = msg.data, time.monotonic()

    # 受信した台車の並進実速度を表示用に保存する。
    def odom(self, msg):
        self.actual_speed = msg.twist.twist.linear.x

    # 有効なLiDAR距離の最小値を画面表示用に保存する。安全な進路の計算は別ノードが担当する。
    def scan(self, msg):
        valid = [v for v in msg.ranges if math.isfinite(v) and msg.range_min <= v <= msg.range_max]
        self.nearest = min(valid, default=math.inf)

    # 並進・旋回の値をTwistに詰めて手動指令トピックへ送る。引数なしなら停止指令。
    def send(self, v=0., w=0.):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.pub.publish(msg)


# 手動操作と自動速度設定を行うQtウィジェット。
class ManualPanel(QtWidgets.QWidget):
    # このクラスで使う状態・通信先・画面部品を初期化する。
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.held_keys = set()
        self.held_buttons = set()
        self.pending = []
        self.last_mode = ''
        self.setWindowTitle('FCTX-Loop Cart - Control Panel')
        self.setMinimumSize(540, 770)
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.setStyleSheet('''
            QWidget { font-family: "Meiryo", "Noto Sans CJK JP"; background: #101b2b; color: #edf4fa; font-size: 15px; }
            QLabel#title { font-size: 25px; font-weight: bold; }
            QLabel#muted { color: #a5b8cd; font-size: 13px; }
            QLabel#status { background: #1c3045; padding: 14px; border-radius: 8px; }
            QPushButton { background: #24394f; border: 1px solid #3b526c; border-radius: 8px; padding: 12px; }
            QDoubleSpinBox { background: #24394f; border: 1px solid #3b526c; border-radius: 5px; padding: 4px; }
            QPushButton:hover { background: #34536e; }
            QPushButton:pressed { background: #137e91; }
            QPushButton:checked { background: #087f8c; border-color: #46d7c6; }
            QPushButton:disabled { color: #62768a; background: #18283b; }
            QPushButton#stop { background: #ae3347; font-size: 18px; font-weight: bold; }
            QSlider::groove:horizontal { background: #344b61; height: 7px; border-radius: 3px; }
            QSlider::handle:horizontal { background: #55ddcb; width: 18px; margin: -6px 0; border-radius: 8px; }
        ''')
        root_layout = QtWidgets.QHBoxLayout(self)
        left = QtWidgets.QWidget()
        left_container=QtWidgets.QWidget();left_column=QtWidgets.QVBoxLayout(left_container)
        left_column.setContentsMargins(0,0,0,0)
        left_scroll=QtWidgets.QScrollArea();left_scroll.setWidgetResizable(True)
        left_scroll.setFrameShape(QtWidgets.QFrame.NoFrame);left_scroll.setMinimumWidth(510)
        left_scroll.setWidget(left);left_column.addWidget(left_scroll,1);root_layout.addWidget(left_container)

        layout = QtWidgets.QVBoxLayout(left)
        layout.setSizeConstraint(QtWidgets.QLayout.SetMinimumSize)
        self.control_layout = layout
        layout.setContentsMargins(24, 20, 24, 20)
        title = QtWidgets.QLabel('FCTX-4輪台車コントロール')
        title.setObjectName('title')
        layout.addWidget(title)
        hint = QtWidgets.QLabel('矢印キー / WASD  •  キーを押している間だけ走行')
        hint.setObjectName('muted')
        layout.addWidget(hint)
        mode_row = QtWidgets.QHBoxLayout()
        self.manual = self.button('手動モード', lambda: self.set_mode(True))
        self.auto = self.button('自動回避', lambda: self.set_mode(False))
        for b in (self.manual, self.auto):
            b.setCheckable(True)
            mode_row.addWidget(b)
        layout.addLayout(mode_row)
        self.status = QtWidgets.QLabel('シミュレーションに接続中…')
        self.status.setObjectName('status')
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        speed_grid = QtWidgets.QGridLayout()
        speed_grid.setHorizontalSpacing(18)
        self.forward_speed = SpeedControl('手動・前進', MAX_FORWARD_SPEED, .40)
        self.reverse_speed = SpeedControl('手動・後退', MAX_REVERSE_SPEED, .25)
        self.turn_speed = SpeedControl('手動・旋回', MAX_TURN_SPEED, .65, 'rad/s')
        self.auto_speed = SpeedControl('自動・速度上限', MAX_AUTO_SPEED, .45)
        for i, control in enumerate((self.forward_speed, self.reverse_speed, self.turn_speed, self.auto_speed)):
            speed_grid.addWidget(control, i//2, i%2)
        layout.addLayout(speed_grid)
        self.speed = self.forward_speed.slider
        self.auto_speed.valueChanged.connect(self.node.set_auto_speed)
        presets = QtWidgets.QHBoxLayout()
        for name, label in [('low', '低速'), ('normal', '標準'), ('fast', '高速')]:
            presets.addWidget(self.button(label, lambda checked=False, n=name: self.apply_preset(n)))
        layout.addLayout(presets)
        self.speed_feedback = QtWidgets.QLabel('自動速度の設定を取得中…')
        self.speed_feedback.setObjectName('muted')
        layout.addWidget(self.speed_feedback)
        grid = QtWidgets.QGridLayout()
        self.directions = {}
        for action, text, row, col in [('forward', '↑ 前進 / W', 0, 1), ('left', '← 左旋回 / A', 1, 0), ('right', '右旋回 / D →', 1, 2), ('backward', '↓ 後退 / S', 2, 1)]:
            b = self.button(text)
            b.setMinimumHeight(56)
            b.pressed.connect(lambda a=action: self.held_buttons.add(a))
            b.released.connect(lambda a=action: self.release_button(a))
            grid.addWidget(b, row, col)
            self.directions[action] = b
        halt = self.button('停止', self.stop)
        grid.addWidget(halt, 1, 1)
        layout.addLayout(grid)
        self.metrics = QtWidgets.QLabel('速度 —    壁まで —')
        self.metrics.setObjectName('muted')
        layout.addWidget(self.metrics)
        self.stop_button = self.button('■ 停止ロック  /  Space', self.stop)
        self.stop_button.setObjectName('stop')
        # 停止ロックはスクロールしても常に見える位置へ固定する。
        left_column.addWidget(self.stop_button)
        layout.addWidget(self.button('停止ロックを解除', self.resume))
        self.notice = QtWidgets.QLabel('手動操作中はこのパネルを選択してください。\n障害物への接近・通信断は自動停止。設定速度より障害物との距離を優先して減速します。')
        self.notice.setWordWrap(True)
        self.notice.setObjectName('muted')
        layout.addWidget(self.notice)
        self.navigation = GoalPanel(node, self)
        # 小さい画面では右側をスクロールし、地図と入力欄が重ならないようにする。
        navigation_scroll = QtWidgets.QScrollArea()
        navigation_scroll.setWidgetResizable(True)
        navigation_scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        navigation_scroll.setMinimumWidth(500)
        navigation_scroll.setWidget(self.navigation)
        root_layout.addWidget(navigation_scroll, 1)
        self.setMinimumWidth(1050)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(50)

    # 前進・後退・旋回・自動上限を選択したプリセットへまとめて変更する。
    def apply_preset(self, name):
        controls = (self.forward_speed, self.reverse_speed, self.turn_speed, self.auto_speed)
        for control, value in zip(controls, PRESETS[name]):
            control.set_value(value)

    # 共通デザインのボタンを作り、キーボード運転のフォーカスを奪わないようにする。
    def button(self, text, callback=None):
        b = QtWidgets.QPushButton(text)
        b.setFocusPolicy(QtCore.Qt.NoFocus)
        if callback:
            b.clicked.connect(callback)
        return b

    # 保持中のキー・方向ボタンを消し、直ちにゼロ指令を送る。
    def clear_input(self):
        self.held_keys.clear()
        self.held_buttons.clear()
        self.node.send()

    # 離したボタンを保持集合から削除して一旦停止する。残りの入力は次の周期で反映する。
    def release_button(self, action):
        self.held_buttons.discard(action)
        self.node.send()  # Stop immediately; next heartbeat applies remaining held keys.

    # サービス接続を確認して非同期要求を送り、応答待ちと完了時の表示文を保存する。
    def request(self, client, value, done_text):
        if not client.service_is_ready():
            self.notice.setText('接続待ちです。シミュレーションの起動を確認してください。')
            return
        self.pending.append((client.call_async(SetBool.Request(data=value)), done_text))

    # 運転入力を解除してから、手動か自動かを安全ゲートへ要求する。
    def set_mode(self, manual):
        self.navigation.abandon_requests()
        self.clear_input()
        if not manual and self.navigation.free_drive():
            return
        self.request(self.node.mode_client, manual, '手動モードに切替えました。' if manual else '現在の向きから障害物を避けて走行します。')

    # 運転入力を解除し、安全ゲートへ停止ロックを要求する。
    def stop(self):
        self.navigation.abandon_requests()
        self.clear_input()
        self.request(self.node.enable_client, False, '停止ロック中。再開するには解除してください。')

    # 古い運転入力を解除してから停止ロックの解除を要求する。
    def resume(self):
        self.clear_input()
        self.request(self.node.enable_client, True, '停止ロックを解除しました。')

    # キーの自動リピートを無視し、手動方向入力または停止操作を処理する。数値編集中は運転入力にしない。
    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return
        if event.key() in KEYS:
            if isinstance(QtWidgets.QApplication.focusWidget(), (QtWidgets.QAbstractSpinBox, QtWidgets.QLineEdit)):
                event.ignore()
                return
            if self.node.mode == 'manual':
                self.held_keys.add(event.key())
            event.accept()
        elif event.key() in (QtCore.Qt.Key_Space, QtCore.Qt.Key_Escape):
            self.stop()
            event.accept()
        else:
            super().keyPressEvent(event)

    # 離したキーを保持集合から削除し、すぐにゼロ指令を送る。
    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return
        self.held_keys.discard(event.key())
        self.node.send()
        event.accept()

    # 画面が非アクティブになったら手動入力を解除し、押しっぱなし状態を残さない。
    def changeEvent(self, event):
        if event.type() == QtCore.QEvent.ActivationChange and not self.isActiveWindow():
            self.clear_input()
        super().changeEvent(event)

    # ROS応答、設定値、接続状態、保持入力を処理して表示を更新し、現在の手動指令を送る。
    def tick(self):
        # Qtを長時間止めないよう、待ち時間ゼロでROSの受信処理を小刻みに進める。
        for _ in range(12):
            rclpy.spin_once(self.node, timeout_sec=0)
        for future, text in list(self.pending):
            if future.done():
                try:
                    result = future.result()
                    self.notice.setText(text if result.success else result.message)
                except Exception as exc:
                    self.notice.setText('通信エラー: '+str(exc))
                self.pending.remove((future, text))
        self.node.poll_auto_speed()
        pending = self.node.pending_speed is not None or self.node.speed_future is not None
        confirmed = self.node.confirmed_auto_speed
        if confirmed is not None and not pending and not self.auto_speed.editing():
            self.auto_speed.set_value(confirmed, emit=False)
        if self.node.speed_error:
            feedback = self.node.speed_error
        elif self.node.pending_speed is not None or (self.node.speed_future is not None and self.node.speed_request[0] == 'set'):
            feedback = '自動速度の変更を反映中…'
        elif confirmed is None:
            feedback = '自動速度の設定を取得中…'
        else:
            feedback = f'自動速度上限 {confirmed:.2f} m/s（障害物に近いと自動減速）'
        self.speed_feedback.setText(feedback)
        # 安全状態の通知が古いときは、画面に入力が残っていても手動指令を停止させる。
        connected = time.monotonic()-self.node.state_time < 1.0
        if self.last_mode != self.node.mode:
            self.clear_input()
            self.last_mode = self.node.mode
        if not connected or not self.isActiveWindow() or self.node.state == 'user_stop':
            self.clear_input()
        manual = self.node.mode == 'manual'
        self.manual.setChecked(manual)
        self.auto.setChecked(self.node.mode == 'auto')
        for button in self.directions.values():
            button.setEnabled(connected and manual)
        actions = {KEYS[key] for key in self.held_keys} | self.held_buttons
        v, w = velocity(actions, self.forward_speed.value(), self.reverse_speed.value(), self.turn_speed.value()) if connected and manual else (0., 0.)
        self.node.send(v, w)
        mission_state = self.navigation.data.get('mission', {}).get('state')
        mode = '手動' if manual else ('配送ミッション' if mission_state in ('running', 'waiting') else ('目的地移動' if self.navigation.data.get('active') else '自動回避'))
        state = STATE_NAMES.get(self.node.state, self.node.state)
        self.status.setText(f'{mode}  |  {state}' if connected else '接続待ち・手動指令は停止中')
        distance = f'{self.node.nearest:.2f} m' if math.isfinite(self.node.nearest) else '—'
        self.metrics.setText(f'実速度 {self.node.actual_speed:+.2f} m/s    LiDAR最短距離 {distance}')

    # 画面の更新を止め、保持入力を解除して停止指令を送る。通常パネル単独ではGazeboを終了しない。
    def closeEvent(self, event):
        self.timer.stop()
        self.clear_input()
        for _ in range(3):
            self.node.send()
        event.accept()


# Qtの操作画面とROS通信。押している間だけ手動指令を送り、自動速度の設定や停止要求も扱う。 起動から終了処理までをまとめる入口。
def main():
    app = QtWidgets.QApplication(sys.argv)
    configure_japanese_font(app)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, lambda *_: app.quit())
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    node = PanelNode()
    panel = ManualPanel(node)
    panel.show()
    panel.activateWindow()
    panel.setFocus()
    snapshot = os.environ.get('LOOP_PANEL_SNAPSHOT')
    if snapshot:
        QtCore.QTimer.singleShot(1500, lambda: panel.grab().save(snapshot))
    try:
        app.exec_()
    finally:
        node.send()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
