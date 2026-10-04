#!/usr/bin/env python3
# ファイルの役割: 操作パネルを残したままGazeboを終了・再起動し、選択したコースへ切り替える。切替後は手動・停止で開始する。
"""Persistent control panel owning one simulation process at a time."""
import os
import argparse
from pathlib import Path
import signal
import subprocess
import sys
import time
import fcntl
import json,hashlib,math
import re
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
import rclpy
from rclpy.signals import SignalHandlerOptions
from PyQt5 import QtCore, QtWidgets
from manual_panel import ManualPanel, PanelNode, configure_japanese_font

ROOT = Path(__file__).resolve().parents[1]
COURSES = [
    ('円形コース', 'loop_course.sdf', '壁に囲まれた基本の周回コース'),
    ('障害物コース', 'obstacle_course.sdf', '箱2個とポール1個を回避'),
    ('楕円コース', 'oval_course.sdf', '長い直線に近い区間と緩いカーブ'),
    ('倉庫コース', 'warehouse_course.sdf', '長方形の広場と4つの棚。自由に障害物を回避'),
    ('大型楕円コース', 'large_oval_course.sdf', '24×16m。広い走行帯の大型周回コース'),
    ('大型倉庫コース', 'large_warehouse_course.sdf', '26×20m。9個の棚と広い通路'),
    ('スラローム障害物コース', 'slalom_course.sdf', '24×16m。交互に配置した3枚の壁を迂回'),
]

# サービスCLIの応答・終了を待ってから、所有launchのSIGINT経路で終了する。
# /server_control stopの別スレッドとSensors破棄が重なる経路は使わない。
SHUTDOWN_SERVICE_TIMEOUT = 5.
SHUTDOWN_SERVICE_KILL_TIMEOUT = 8.
SHUTDOWN_SLAM_STARTUP_TIMEOUT = 20.
SHUTDOWN_LAUNCH_TERM_TIMEOUT = 40.
SHUTDOWN_LAUNCH_KILL_TIMEOUT = 45.


# ManualPanelを拡張し、コースと子プロセスの寿命を管理する。
class CoursePanel(ManualPanel):
    # このクラスで使う状態・通信先・画面部品を初期化する。
    def __init__(self, node, simulation_gui=True):
        super().__init__(node)
        self.setWindowTitle('FCTX-Loop Cart - Navigation')
        self.setMinimumSize(1100, 890)
        self.simulation_gui = simulation_gui
        self.active_pose_source='simulation'
        self.map_save_process=None
        self.process = None
        self.log = None
        self.next_course = None
        self.stopping_since = None
        self.stop_signal_sent = False
        self.stop_signal_since = None
        self.stop_term_sent = False
        self.stop_kill_sent = False
        self.shutdown_request = None
        self.shutdown_request_since = None
        self.shutdown_request_term_sent = False
        self.shutdown_request_kill_sent = False
        self.shutdown_request_kind = None
        self.shutdown_acknowledged = False
        self.shutdown_pause_confirmed = False
        self.shutdown_slam_deactivation_started = False
        self.shutdown_slam_deactivated = False
        self.shutdown_startup_wait_since = None
        self.shutdown_events = []
        self.closing = False
        self.active_course = None
        self.starting = False
        self.startup_since = 0.
        self.startup_retries = 0
        self.startup_failure = None
        self.combo = QtWidgets.QComboBox()
        for label, filename, description in COURSES:
            self.combo.addItem(label, filename)
        self.combo.setCurrentIndex(next(i for i,c in enumerate(COURSES) if c[1] == 'oval_course.sdf'))
        row = QtWidgets.QHBoxLayout()
        row.addWidget(self.combo, 1)
        self.load_button = self.button('選択したコースを開始', self.switch_course)
        row.addWidget(self.load_button)
        row.addWidget(self.button('コース終了', self.stop_course))
        self.course_status = QtWidgets.QLabel('コースを選択してください（開始時は手動・停止）')
        self.course_status.setWordWrap(True)
        self.control_layout.insertLayout(2, row)
        self.control_layout.insertWidget(3, self.course_status)
        self.pose_source=QtWidgets.QComboBox()
        for label,value in [('シミュレーション位置','simulation'),('SLAM 地図作成','slam'),('保存地図で自己位置推定','localization')]:self.pose_source.addItem(label,value)
        source_row=QtWidgets.QHBoxLayout();source_row.addWidget(self.pose_source,1)
        source_row.addWidget(self.button('地図を保存',self.save_map))
        self.control_layout.insertLayout(4,source_row)
        initial_row=QtWidgets.QHBoxLayout()
        self.initial_yaw=QtWidgets.QDoubleSpinBox();self.initial_yaw.setRange(-180,180);self.initial_yaw.setSuffix(' °')
        initial_row.addWidget(QtWidgets.QLabel('推定初期方向'));initial_row.addWidget(self.initial_yaw)
        initial_row.addWidget(self.button('選択位置を初期位置に',self.set_initial_pose))
        self.control_layout.insertLayout(5,initial_row)
        self.localization_status=QtWidgets.QLabel('位置モードを選択してコースを開始してください')
        self.localization_status.setWordWrap(True);self.control_layout.insertWidget(6,self.localization_status)
        self.initial_pub=node.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
        node.create_subscription(String,'/loop/localization_status',lambda m:self.localization_status.setText(m.data),10)
        self.combo.currentIndexChanged.connect(self.describe)
        self.manager = QtCore.QTimer(self)
        self.manager.timeout.connect(self.poll_process)
        self.manager.start(100)

    def set_initial_pose(self):
        if self.active_pose_source!='localization':
            self.localization_status.setText('保存地図で自己位置推定モードのときに指定できます');return
        self.set_mode(True)
        msg=PoseWithCovarianceStamped();msg.header.frame_id='map'
        msg.pose.pose.position.x=self.navigation.x.value();msg.pose.pose.position.y=self.navigation.y.value()
        yaw=math.radians(self.initial_yaw.value());msg.pose.pose.orientation.z=math.sin(yaw/2);msg.pose.pose.orientation.w=math.cos(yaw/2)
        msg.pose.covariance[0]=msg.pose.covariance[7]=.25;msg.pose.covariance[35]=.25
        self.initial_pub.publish(msg)
        self.localization_status.setText('初期位置を送信しました。推定が安定してから開始してください')

    def save_map(self):
        if self.active_pose_source!='slam' or self.process is None:
            self.localization_status.setText('SLAM地図作成モードで走行してから保存してください');return
        if self.map_save_process is not None:return
        process=QtCore.QProcess(self);self.map_save_process=process
        process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        def finished(*_):
            lines=bytes(process.readAllStandardOutput()).decode('utf-8',errors='replace').strip().splitlines()
            self.localization_status.setText(lines[-1] if lines else '地図保存が終了しました')
            self.map_save_process=None;process.deleteLater()
        process.finished.connect(finished)
        process.start('/usr/bin/python3',[str(ROOT/'scripts/save_slam_map.py'),str(ROOT/'worlds'/COURSES[self.active_course][1])])
        self.localization_status.setText('地図を保存しています…')

    # 選択中コースの説明と、切替によって位置がリセットされることを表示する。
    def describe(self):
        if self.stopping_since is None:
            self.course_status.setText(COURSES[self.combo.currentIndex()][2]+' ／切替時に位置をリセット')

    # 選択を次の起動候補として保存し、実行中コースの終了を開始する。
    def switch_course(self):
        if self.map_save_process is not None:
            self.course_status.setText('地図保存が終わってからコースを切り替えてください');return
        if self.closing or self.stopping_since is not None:
            return
        self.startup_retries = 0
        self.startup_failure = None
        self.next_course = self.combo.currentIndex()
        self.stop_process()
        if self.process is None:
            self.start_pending()

    # 次の起動予約を消して、現在のシミュレーションだけを終了する。
    def stop_course(self):
        self.next_course = None
        self.startup_failure = None
        self.stop_process()

    # pause確認後、SLAMは内部スレッドの停止要求を回収してから所有launchを終了する。
    def stop_process(self):
        self.clear_input()
        if self.process is not None and self.process.poll() is None:
            self.stop()
            if self.stopping_since is None:
                self.stopping_since = time.monotonic()
                # 初期化待機やサービス回収中の再押下で、運転・停止解除を再開させない。
                # QWidgetの入力だけを止め、Qt timersとROS受信による終了監視は継続する。
                self.setEnabled(False)
                direct_cancel = self.starting and self.active_pose_source != 'slam'
                self.stop_signal_sent = direct_cancel
                self.stop_signal_since = self.stopping_since if direct_cancel else None
                self.stop_term_sent = False
                self.stop_kill_sent = False
                self.shutdown_acknowledged = False
                self.shutdown_pause_confirmed = False
                self.shutdown_slam_deactivation_started = False
                self.shutdown_slam_deactivated = False
                self.shutdown_startup_wait_since = None
                self.shutdown_events = []
                if self.starting:
                    if direct_cancel:
                        self.signal_owned(signal.SIGINT)
                    else:
                        # 初期化途中のdeactivate拒否直後にSIGINTを重ねない。
                        # 位置・地図・安全監視の準備完了を待つが、無応答でも有限時間で停止へ進む。
                        self.shutdown_startup_wait_since = self.stopping_since
                        self.shutdown_events.append(('slam_startup_wait',self.stopping_since))
                else:
                    # 更新中の描画を先に止め、別stopThreadを作らないSIGINT経路へ集める。
                    self.start_shutdown_request('pause',['gz','service','-s','/world/loop_course/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','2000','--req','pause: true'])
            self.load_button.setEnabled(False)
            self.course_status.setText('コースを終了しています…')

    def signal_owned(self, sig, process_group=False):
        """このパネルが起動したlaunchだけを対象にし、終了との競合は無視する。"""
        try:
            (os.killpg if process_group else os.kill)(self.process.pid, sig)
            self.shutdown_events.append((signal.Signals(sig).name,time.monotonic()))
        except ProcessLookupError:
            pass

    def interrupt_owned_launch(self):
        """通常終了も無応答時も、SIGINTは一度だけ所有launchへ送る。"""
        if self.stop_signal_sent or self.process is None or self.process.poll() is not None:return
        self.stop_signal_sent=True
        self.stop_signal_since=time.monotonic()
        self.signal_owned(signal.SIGINT)

    def prepare_launch_interrupt(self):
        """所有する旧コースのSLAMだけをdeactivateし、全体SIGINTを後へ回す。"""
        if self.process is None or self.process.poll() is not None:return
        if self.active_pose_source == 'slam' and not self.shutdown_slam_deactivation_started:
            self.shutdown_slam_deactivation_started = True
            self.start_shutdown_request('slam_deactivate', [
                'ros2', 'service', 'call', '/slam_toolbox/change_state',
                'lifecycle_msgs/srv/ChangeState', '{transition: {id: 4}}',
            ])
        else:
            self.interrupt_owned_launch()

    def start_shutdown_request(self, kind, command):
        env=os.environ.copy();env.setdefault('GZ_PARTITION','ros2_gazebo_loop')
        env.setdefault('ROS_DOMAIN_ID','42')
        try:
            self.shutdown_request=subprocess.Popen(command,env=env,stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        except OSError:
            # CLIを起動できなくても、所有launchの終了依頼と有限時間の監視は続ける。
            self.shutdown_request=None
            self.shutdown_events.append((kind+'_unavailable',time.monotonic()))
            # pause/statsが使えなくても、稼働中SLAMの停止は一度だけ試す。
            self.prepare_launch_interrupt()
            return
        self.shutdown_request_kind=kind
        self.shutdown_request_since=time.monotonic()
        self.shutdown_request_term_sent=False
        self.shutdown_request_kill_sent=False
        self.shutdown_events.append((kind+'_requested',self.shutdown_request_since))

    def poll_shutdown_request(self):
        """サービスCLI自体の終了も確認し、遅れた停止要求が次コースへ届くのを防ぐ。"""
        request=self.shutdown_request
        if request is None:return
        if request.poll() is not None:
            timed_out = self.shutdown_request_term_sent or self.shutdown_request_kill_sent
            output=request.communicate()[0].decode('utf-8',errors='replace')
            if self.log is not None:
                self.log.write(output);self.log.flush()
            kind=self.shutdown_request_kind
            self.shutdown_request=None
            self.shutdown_request_kind=None
            # 所有launchが既に終了していたら、古いpause要求の後に新しい要求を発行しない。
            if self.process is None or self.process.poll() is not None:return
            if kind=='pause':
                self.shutdown_acknowledged=request.returncode == 0 and any(line.strip() == 'data: true' for line in output.splitlines())
                if self.shutdown_acknowledged:
                    self.shutdown_events.append(('pause_acknowledged',time.monotonic()))
                    self.start_shutdown_request('stats',['gz','topic','-e','-t','/world/loop_course/stats','-n','1','--json-output'])
                    return
            elif kind=='stats' and request.returncode == 0:
                # CLIは-n 1でも同時受信した複数JSON行を出すことがある。
                for line in output.splitlines():
                    try:
                        self.shutdown_pause_confirmed |= json.loads(line).get('paused') is True
                    except (ValueError,AttributeError):
                        continue
                if self.shutdown_pause_confirmed:
                    self.shutdown_events.append(('pause_applied',time.monotonic()))
            elif kind=='slam_deactivate':
                self.shutdown_slam_deactivated = request.returncode == 0 and bool(
                    re.search(r'\bsuccess\s*(?:=\s*True\b|:\s*true\b)', output))
                outcome = ('acknowledged' if self.shutdown_slam_deactivated else
                           'timed_out' if timed_out else 'rejected' if request.returncode == 0 else 'failed')
                self.shutdown_events.append(('slam_deactivate_'+outcome,time.monotonic()))
                self.interrupt_owned_launch()
                return
            self.prepare_launch_interrupt()
            return
        elapsed=time.monotonic()-self.shutdown_request_since
        try:
            if elapsed>SHUTDOWN_SERVICE_KILL_TIMEOUT and not self.shutdown_request_kill_sent:
                request.kill();self.shutdown_request_kill_sent=True
            elif elapsed>SHUTDOWN_SERVICE_TIMEOUT and not self.shutdown_request_term_sent:
                request.terminate();self.shutdown_request_term_sent=True
        except ProcessLookupError:
            pass

    # 古い入力・通信状態を消し、予約したコースを手動停止で起動する。操作画面は新しく起動しない。
    def start_pending(self):
        if (self.next_course is None or self.closing or self.process is not None
                or self.shutdown_request is not None or self.shutdown_startup_wait_since is not None):
            return
        index, self.next_course = self.next_course, None
        label, filename, _ = COURSES[index]
        source=self.pose_source.currentData()
        map_file=ROOT/'maps'/Path(filename).stem/'map.yaml'
        if source=='localization':
            try:
                meta=json.loads(map_file.with_name('map_metadata.json').read_text())
                if meta['world_sha256']!=hashlib.sha256((ROOT/'worlds'/filename).read_bytes()).hexdigest() or not map_file.is_file():raise ValueError()
            except (OSError,ValueError,KeyError):
                self.course_status.setText('このコースの保存地図がありません。SLAMモードで地図を保存してください');return
        self.active_pose_source=source
        self.localization_status.setText('シミュレーション位置' if source=='simulation' else '地図・位置推定の起動待ち')
        self.clear_input()
        self.node.state_time = 0.
        self.navigation.reset()
        self.node.actual_speed = 0.
        self.node.nearest = float('inf')
        self.node.mode = ''
        self.node.state = ''
        self.node.speed_error = ''
        self.node.pending_speed = None
        self.node.confirmed_auto_speed = None
        self.node.speed_future = None
        self.node.last_speed_query = 0.
        self.pending.clear()
        self.log = (ROOT/'logs/course_selector.log').open('a')
        env = os.environ.copy()
        env.pop('QT_QPA_PLATFORM', None)
        self.process = subprocess.Popen([
            'bash', str(ROOT/'run_loop.sh'), 'controls:=false', 'mode:=manual',
            'gui:='+str(self.simulation_gui).lower(),f'pose_source:={source}',f'map_file:={map_file}',
            f'world:={ROOT/"worlds"/filename}', f'auto_speed:={self.auto_speed.value():.2f}'
        ], cwd=ROOT, env=env, stdin=subprocess.DEVNULL, stdout=self.log,
            stderr=subprocess.STDOUT, start_new_session=True)
        (ROOT/'logs/loop_gui.pid').write_text(str(self.process.pid))
        self.active_course = index
        self.starting = True
        self.startup_since = time.monotonic()
        self.course_status.setText(label+' 起動中／手動で停止待機。自動回避ボタンで走行')

    def startup_ready(self):
        """プロセスの存在だけでなく、新しい制御・位置・地図の受信で起動完了を判定する。"""
        data=self.navigation.data
        fresh=time.monotonic()-self.node.state_time < .5 and time.monotonic()-self.navigation.last_received < 1.
        expected=str(ROOT/'worlds'/COURSES[self.active_course][1])
        return (fresh and bool(data.get('pose')) and self.navigation.world==expected
                and data.get('pose_source')==self.active_pose_source
                and self.node.state not in ('waiting_for_fresh_data','waiting_for_scan')
                and (self.active_pose_source=='simulation' or self.navigation.map.map_image is not None))

    # 100msごとに子プロセスの状態を確認する。終了後に次のコースを起動し、終了待ち超過には強いシグナルを送る。
    def poll_process(self):
        self.poll_shutdown_request()
        if self.process is not None:
            code = self.process.poll()
            if code is not None:
                # 古い終了サービスが次のコースへ届かないよう、要求の完了まで再起動しない。
                if self.shutdown_request is not None:return
                stopping = self.stopping_since is not None
                failed_start = self.starting and (not stopping or self.startup_failure is not None)
                # 起動中だけ一度再試行する。走行中の異常終了では自動再起動しない。
                retry = failed_start and self.startup_retries < 1 and not self.closing and self.next_course is None
                if retry:
                    self.startup_retries += 1
                    self.next_course = self.active_course
                    self.pose_source.setCurrentIndex(self.pose_source.findData(self.active_pose_source))
                failure = self.startup_failure
                self.startup_failure = None
                self.starting = False
                self.process = None
                self.log.close()
                self.log = None
                self.stopping_since = None
                self.shutdown_startup_wait_since = None
                self.load_button.setEnabled(True)
                self.node.state_time = 0.
                self.course_status.setText(('起動確認に失敗しました: '+failure if failure else '起動に失敗しました' if failed_start else 'コース終了' if stopping else '実行プロセスが終了しました。コース開始で再起動できます')+' ／ログ: logs/course_selector.log')
                if self.closing:
                    self.close()
                else:
                    self.start_pending()
                    self.setEnabled(True)
            elif self.starting and self.stopping_since is None and self.startup_ready():
                self.starting = False
                self.course_status.setText(COURSES[self.active_course][0]+' 読み込み済み／手動・自動回避を選んで操作できます')
            elif self.starting and self.stopping_since is None and time.monotonic()-self.startup_since > 60.:
                self.startup_failure = '60秒以内にセンサー・位置推定の準備が完了しませんでした'
                self.stop_process()
            elif self.stopping_since is not None:
                if self.shutdown_startup_wait_since is not None:
                    # 起動途中のSLAM停止だけに適用し、入力停止・次コース保留は維持する。
                    ready = self.startup_ready()
                    expired = time.monotonic()-self.shutdown_startup_wait_since >= SHUTDOWN_SLAM_STARTUP_TIMEOUT
                    if ready or expired:
                        self.shutdown_startup_wait_since = None
                        self.shutdown_events.append(('slam_startup_ready' if ready else 'slam_startup_wait_timed_out',time.monotonic()))
                        self.prepare_launch_interrupt()
                    return
                # pause・stats CLIを回収するまで、次コース開始や別の終了経路へ移らない。
                if self.shutdown_request is not None:return
                if not self.stop_signal_sent:
                    self.interrupt_owned_launch()
                elif self.stop_signal_since is not None:
                    since_signal=time.monotonic()-self.stop_signal_since
                    if since_signal>SHUTDOWN_LAUNCH_KILL_TIMEOUT and not self.stop_kill_sent:
                        self.stop_kill_sent=True;self.signal_owned(signal.SIGKILL,process_group=True)
                    elif since_signal>SHUTDOWN_LAUNCH_TERM_TIMEOUT and not self.stop_term_sent:
                        self.stop_term_sent=True;self.signal_owned(signal.SIGTERM,process_group=True)
        elif self.closing:
            self.close()

    # 所有するGazeboが終わるまで画面終了を保留し、子プロセスを残さないようにする。
    def closeEvent(self, event):
        if self.process is not None:
            self.closing = True
            self.next_course = None
            self.stop_process()
            event.ignore()
        else:
            self.manager.stop()
            super().closeEvent(event)


# 操作パネルを残したままGazeboを終了・再起動し、選択したコースへ切り替える。切替後は手動・停止で開始する。 起動から終了処理までをまとめる入口。
def main():
    parser = argparse.ArgumentParser(description='コース選択と目的地移動の操作画面')
    parser.add_argument('--pose-source', choices=['simulation','slam','localization'],default='simulation')
    parser.add_argument('--start', action='store_true', help='選択コースを手動停止で読み込む')
    parser.add_argument('--course', choices=[c[1] for c in COURSES], default='oval_course.sdf')
    args = parser.parse_args()
    ROOT.joinpath('logs').mkdir(exist_ok=True)
    lock = (ROOT/'logs/course_selector.lock').open('w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print('Course selection is already running.', flush=True)
        return
    app = QtWidgets.QApplication(sys.argv)
    configure_japanese_font(app)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = PanelNode()
    panel = CoursePanel(node)
    panel.pose_source.setCurrentIndex(panel.pose_source.findData(args.pose_source))
    panel.combo.setCurrentIndex(next(i for i, c in enumerate(COURSES) if c[1] == args.course))
    signal.signal(signal.SIGTERM, lambda *_: panel.close())
    signal.signal(signal.SIGINT, lambda *_: panel.close())
    # WSLgの複数画面環境でも、起動時は主画面内に操作パネルを配置する。
    def show_on_primary():
        screen = app.primaryScreen()
        panel.showNormal()
        if screen is not None:
            area = screen.availableGeometry()
            panel.move(area.left() + 60, area.top() + 60)
        panel.raise_()
        panel.activateWindow()
    panel.show()
    QtCore.QTimer.singleShot(500, show_on_primary)
    panel.setFocus()
    snapshot = os.environ.get('LOOP_PANEL_SNAPSHOT')
    if snapshot:
        QtCore.QTimer.singleShot(8000, lambda: panel.grab().save(snapshot))
    if args.start:
        QtCore.QTimer.singleShot(200, panel.switch_course)
    try:
        app.exec_()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        lock.close()


if __name__ == '__main__':
    main()
