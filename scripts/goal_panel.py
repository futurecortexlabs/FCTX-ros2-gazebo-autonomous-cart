"""目的地座標、クリックできる地図、計画経路と到着状態を表示するQt部品。"""
import json
import math
import time
from pathlib import Path
from PyQt5 import QtCore, QtGui, QtWidgets
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy
import numpy as np
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from world_clearance import WorldClearance
from mission_panel import MissionPanel
from map_cache import occupancy_signature


class GoalMap(QtWidgets.QWidget):
    selected = QtCore.pyqtSignal(float, float)

    def __init__(self):
        super().__init__()
        self.setMinimumSize(350, 280)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.shapes, self.data, self.selection = [], {}, None
        self.bounds = (-9., 9., -7., 7.)
        self.map_image=None
        self.map_rectangle=None

    def transform(self):
        xmin, xmax, ymin, ymax = self.bounds
        scale = min((self.width()-36)/(xmax-xmin), (self.height()-36)/(ymax-ymin))
        return scale, self.width()/2-scale*(xmin+xmax)/2, self.height()/2+scale*(ymin+ymax)/2

    def point(self, x, y):
        s, ox, oy = self.transform()
        return QtCore.QPointF(ox+s*x, oy-s*y)

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton and (self.shapes or self.map_image is not None):
            s, ox, oy = self.transform()
            self.selected.emit(round((event.x()-ox)/s, 2), round((oy-event.y())/s, 2))

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.fillRect(self.rect(), QtGui.QColor('#172638'))
        if self.map_image is not None and self.map_rectangle is not None:
            xmin,xmax,ymin,ymax=self.map_rectangle
            p.drawImage(QtCore.QRectF(self.point(xmin,ymax),self.point(xmax,ymin)),self.map_image)
        p.setPen(QtGui.QPen(QtGui.QColor('#2b4055'), 1))
        for x in range(math.ceil(self.bounds[0]), math.floor(self.bounds[1])+1):
            p.drawLine(self.point(x, self.bounds[2]), self.point(x, self.bounds[3]))
        for y in range(math.ceil(self.bounds[2]), math.floor(self.bounds[3])+1):
            p.drawLine(self.point(self.bounds[0], y), self.point(self.bounds[1], y))
        scale, _, _ = self.transform()
        p.setPen(QtCore.Qt.NoPen)
        p.setBrush(QtGui.QColor('#a7b8c8'))
        for x, y, a, hx, hy, kind in self.shapes:
            p.save();p.translate(self.point(x, y));p.rotate(-math.degrees(a))
            if kind == 'circle':
                p.drawEllipse(QtCore.QPointF(0, 0), hx*scale, hx*scale)
            else:
                p.drawRect(QtCore.QRectF(-hx*scale, -hy*scale, 2*hx*scale, 2*hy*scale))
            p.restore()
        # LiDARで新しく検知した障害物を橙色で示す。
        p.setBrush(QtGui.QColor('#ff8844'));p.setPen(QtCore.Qt.NoPen)
        for x,y in self.data.get('dynamic_obstacles',[]):
            p.drawEllipse(self.point(x,y),max(2.,.23*scale),max(2.,.23*scale))
        path = self.data.get('path', [])
        p.setPen(QtGui.QPen(QtGui.QColor('#55ddcb'), 2))
        for a, b in zip(path, path[1:]):
            p.drawLine(self.point(*a), self.point(*b))
        for goal, color in [(self.data.get('goal'), '#efbfff'), (self.selection, '#ffcd65')]:
            if goal:
                q = self.point(*goal);p.setPen(QtGui.QPen(QtGui.QColor(color), 2));p.setBrush(QtCore.Qt.NoBrush)
                p.drawEllipse(q, 7, 7);p.drawLine(q+QtCore.QPointF(-10,0),q+QtCore.QPointF(10,0));p.drawLine(q+QtCore.QPointF(0,-10),q+QtCore.QPointF(0,10))
        pose = self.data.get('pose')
        if pose:
            x,y,yaw=pose;p.save();p.translate(self.point(x,y));p.rotate(-math.degrees(yaw))
            p.setBrush(QtGui.QColor('#4faaff'));p.setPen(QtCore.Qt.NoPen)
            p.drawRect(QtCore.QRectF(-.45*scale,-.39*scale,.9*scale,.78*scale))
            p.setPen(QtGui.QPen(QtGui.QColor('white'),2));p.drawLine(QtCore.QPointF(0,0),QtCore.QPointF(.7*scale,0));p.restore()
        p.setPen(QtGui.QColor('#bdd0e0'));p.drawText(10,20,'上: +Y   右: +X   1目盛: 1m')


class GoalPanel(QtWidgets.QWidget):
    def __init__(self, node, owner):
        super().__init__()
        self.node, self.owner = node, owner
        self.data, self.last_received, self.world = {}, 0., None
        self.last_grid=None
        self._grid_signature=None
        self.pending = []
        self.mode_after_goal = False
        self.pending_goal = None
        self.request_started = 0.
        layout = QtWidgets.QVBoxLayout(self)
        title = QtWidgets.QLabel('目的地への移動');title.setObjectName('title');layout.addWidget(title)
        self.hint=hint=QtWidgets.QLabel('地図をクリック、またはX・Yを入力してください。\nコースの地図とシミュレーション位置を使用します。')
        hint.setWordWrap(True);hint.setObjectName('muted');layout.addWidget(hint)
        self.map=GoalMap();layout.addWidget(self.map,1)
        coords=QtWidgets.QHBoxLayout();self.x=QtWidgets.QDoubleSpinBox();self.y=QtWidgets.QDoubleSpinBox()
        for label,spin in [('X',self.x),('Y',self.y)]:
            coords.addWidget(QtWidgets.QLabel(label));spin.setRange(-20,20);spin.setDecimals(2);spin.setSingleStep(.1);spin.setSuffix(' m');coords.addWidget(spin)
            spin.valueChanged.connect(self.selection_changed)
        layout.addLayout(coords);self.map.selected.connect(self.select)
        obstacle_row=QtWidgets.QHBoxLayout()
        self.obstacle_add=owner.button('選択位置に箱を追加',lambda:self.obstacle_action(False));obstacle_row.addWidget(self.obstacle_add)
        self.obstacle_remove=owner.button('追加した箱を削除',lambda:self.obstacle_action(True));obstacle_row.addWidget(self.obstacle_remove)
        layout.addLayout(obstacle_row)
        self.obstacle_notice=QtWidgets.QLabel('箱は台車から2.5m以上（走行中は追加の余裕）離してください。')
        self.obstacle_notice.setWordWrap(True);layout.addWidget(self.obstacle_notice)
        self.obstacle_process=None
        tabs=QtWidgets.QTabWidget();layout.addWidget(tabs)
        single=QtWidgets.QWidget();single_layout=QtWidgets.QVBoxLayout(single);tabs.addTab(single,'単一目的地')
        self.start=owner.button('この目的地へ移動',self.go);single_layout.addWidget(self.start)
        single_layout.addWidget(owner.button('目的地を取消して停止',self.cancel))
        self.status=QtWidgets.QLabel('接続待ち');self.status.setWordWrap(True);self.status.setMinimumHeight(65);single_layout.addWidget(self.status)
        legend=QtWidgets.QLabel('青: 台車　緑: 計画経路　黄色: 選択位置\n到着範囲: 0.18m以内 / 目的地走行は最大0.55m/s\n左の「自動回避」は目的地を解除して自由走行します。')
        legend.setWordWrap(True);legend.setObjectName('muted');single_layout.addWidget(legend)
        # FakeNodeを使う従来UIテストでは通信だけを省き、部品の配置は共通にする。
        self.connected_node = hasattr(node, 'create_publisher')
        if self.connected_node:
            self.pub=node.create_publisher(PoseStamped,'/loop/goal',1)
            self.cancel_client=node.create_client(Trigger,'/loop/cancel_goal')
            self.free_client=node.create_client(Trigger,'/loop/free_drive')
            node.create_subscription(String,'/loop/navigation_status',self.receive,1)
            node.create_subscription(OccupancyGrid,'/map',self.receive_grid,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.mission=MissionPanel(self);tabs.addTab(self.mission,'配送ミッション')
        self.timer=QtCore.QTimer(self);self.timer.timeout.connect(self.refresh);self.timer.start(100)

    def receive_grid(self,msg):
        self.last_grid=msg
        if self.data.get('frame')!='map' or not msg.info.width:return
        signature=occupancy_signature(msg)
        if signature==self._grid_signature and self.map.map_image is not None:return
        w,h=msg.info.width,msg.info.height
        values=np.asarray(msg.data,dtype=np.int8).reshape(h,w)
        pixels=np.ascontiguousarray(np.flipud(np.where(values<0,100,np.where(values>=50,20,210))).astype(np.uint8))
        self._grid_signature=signature
        self.map.map_image=QtGui.QImage(pixels.data,w,h,w,QtGui.QImage.Format_Grayscale8).copy()
        x,y=msg.info.origin.position.x,msg.info.origin.position.y;res=msg.info.resolution
        self.map.map_rectangle=(x,x+w*res,y,y+h*res);self.map.shapes=[];self.map.update()

    def obstacle_action(self,remove):
        if self.data.get('frame','world')!='world':
            self.obstacle_notice.setText('箱の追加操作はシミュレーション位置モードで使用してください');return
        if self.obstacle_process is not None:return
        process=QtCore.QProcess(self);self.obstacle_process=process
        args=[str(Path(__file__).with_name('obstacle_control.py'))]
        args+=['--remove-all'] if remove else ['--x',str(self.x.value()),'--y',str(self.y.value())]
        process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        def finished(*_):
            self.obstacle_notice.setText(bytes(process.readAllStandardOutput()).decode('utf-8',errors='replace').strip())
            self.obstacle_process=None;process.deleteLater()
        process.finished.connect(finished)
        process.start('/usr/bin/python3',args)
        self.obstacle_notice.setText('Gazeboへ反映しています…')

    def select(self,x,y):
        self.x.setValue(x);self.y.setValue(y)

    def selection_changed(self):
        self.map.selection=(self.x.value(),self.y.value());self.map.update()

    def abandon_requests(self):
        # 新しい手動・停止操作を優先し、遅い応答で自動モードへ戻さない。
        if hasattr(self,'mission'):self.mission.abandon()
        self.mode_after_goal=False
        self.pending_goal=None
        self.request_started=0.
        for future, _ in self.pending:
            if not future.done():
                future.cancel()
        self.pending.clear()

    def reset(self):
        self.data={};self.last_received=0.;self.world=None;self.map.data={};self.map.shapes=[]
        self.map.map_image=None;self.map.map_rectangle=None;self.last_grid=None;self._grid_signature=None
        self.abandon_requests()
        self.mission.reset()
        self.x.setValue(0.);self.y.setValue(0.)
        self.map.selection=None
        self.map.update()

    def receive(self,msg):
        try:data=json.loads(msg.data)
        except (ValueError,TypeError):return
        self.data,self.last_received=data,time.monotonic()
        for button in (self.obstacle_add,self.obstacle_remove):button.setEnabled(data.get('frame','world')=='world')
        self.hint.setText('地図をクリック、またはX・Yを入力してください。\n'+('実測地図とSLAM/AMCLの推定位置を使用。灰色は未観測です。' if data.get('frame')=='map' else 'コースの地図とシミュレーション位置を使用します。'))
        if data.get('world')!=self.world:
            self.world=data.get('world');self.mode_after_goal=False
            root=Path(__file__).resolve().parents[1]/'worlds'
            path=Path(self.world).resolve() if self.world else None
            if data.get('frame','world')=='world' and path and path.parent==root.resolve() and path.is_file():
                self.map.shapes=WorldClearance(path).shapes
                self.map.bounds=tuple(data.get('bounds',(-9,9,-7,7)))
        if data.get('frame')=='map' and self.map.map_image is None and self.last_grid is not None:self.receive_grid(self.last_grid)
        if data.get('bounds'):
            self.map.bounds=tuple(data['bounds'])
            # 大型コースのmap座標は開始位置から20mを超えるため、地図に合わせて入力範囲を広げる。
            xmin,xmax,ymin,ymax=self.map.bounds
            if all(math.isfinite(v) for v in self.map.bounds):
                self.x.setRange(min(self.x.minimum(),xmin),max(self.x.maximum(),xmax))
                self.y.setRange(min(self.y.minimum(),ymin),max(self.y.maximum(),ymax))
        self.map.data=data;self.map.update()
        # バックエンドが目的地を受理した状態を確認してから自動モードへ切り替える。
        if self.mode_after_goal and data.get('goal')==self.pending_goal and data.get('state') in ('navigating','paused','arrived','rejected'):
            self.mode_after_goal=False
            if data['state']!='rejected':
                self.owner.clear_input()
                self.owner.request(self.node.mode_client,False,'目的地への移動を開始しました。停止ロック中は解除してください。')

    def go(self):
        if not self.connected_node or time.monotonic()-self.last_received>1.:
            self.status.setText('接続を確認してください');return
        self.abandon_requests()
        self.owner.clear_input()
        msg=PoseStamped();msg.header.frame_id=self.data.get('frame','world');msg.pose.position.x=self.x.value();msg.pose.position.y=self.y.value();msg.pose.orientation.w=1.
        self.pending_goal=[self.x.value(),self.y.value()]
        self.request_started=time.monotonic()
        self.mode_after_goal=True;self.pub.publish(msg)
        self.status.setText('経路を計画しています…')

    def cancel(self):
        self.abandon_requests();self.owner.clear_input()
        if self.connected_node and self.cancel_client.service_is_ready():
            self.pending.append((self.cancel_client.call_async(Trigger.Request()),False))
        else:self.owner.stop()

    def free_drive(self):
        self.abandon_requests()
        if self.connected_node and self.free_client.service_is_ready():
            self.pending.append((self.free_client.call_async(Trigger.Request()),True));return True
        return False

    def refresh(self):
        for future,start_auto in list(self.pending):
            if future.done():
                self.pending.remove((future,start_auto))
                try:
                    if future.result().success and start_auto:
                        self.owner.request(self.node.mode_client,False,'自由回避を開始しました。')
                except Exception:self.status.setText('通信に失敗しました')
        live=time.monotonic()-self.last_received<1.
        self.start.setEnabled(live and not self.mode_after_goal)
        if self.mode_after_goal:
            if time.monotonic()-self.request_started > 5.:
                self.mode_after_goal=False
                self.status.setText('目的地の応答を確認できませんでした。再指定してください')
            else:
                self.status.setText('目的地の受理を確認しています…')
            return
        if not live:self.status.setText('目的地制御に接続待ち');return
        d=self.data.get('distance')
        text=self.data.get('detail','')
        if d is not None:text+=f'\n目的地まで {d:.2f} m（直線距離）'
        self.status.setText(text)
