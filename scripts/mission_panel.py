"""配送ルート編集・保存・読込と、応答ID付きの実行操作を行う。"""
import json,time,uuid
from pathlib import Path
from PyQt5 import QtCore,QtWidgets
from std_msgs.msg import String
from mission_control import validate_route


class MissionPanel(QtWidgets.QWidget):
    def __init__(self,goal):
        super().__init__();self.goal=goal;self.pending=None;self.sent=0.
        layout=QtWidgets.QVBoxLayout(self)
        self.table=QtWidgets.QTableWidget(0,3);self.table.setHorizontalHeaderLabels(['X (m)','Y (m)','待機 (秒)'])
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows);self.table.setMinimumHeight(120);self.table.setMaximumHeight(145)
        layout.addWidget(self.table)
        row=QtWidgets.QHBoxLayout()
        for label,action in [('選択位置を追加',self.add),('削除',self.remove),('↑',lambda:self.move(-1)),('↓',lambda:self.move(1))]:row.addWidget(goal.owner.button(label,action))
        layout.addLayout(row)
        row=QtWidgets.QHBoxLayout()
        for label,action in [('保存',self.save),('読込',self.load),('開始',lambda:self.send('start')),('一時停止',lambda:self.send('pause')),('再開',lambda:self.send('resume')),('取消',lambda:self.send('cancel'))]:row.addWidget(goal.owner.button(label,action))
        layout.addLayout(row)
        self.status=QtWidgets.QLabel('地図で地点を選択し、追加してください。待機時間は表で編集できます。')
        self.status.setWordWrap(True);layout.addWidget(self.status)
        if goal.connected_node:self.pub=goal.node.create_publisher(String,'/loop/mission_command',10)
        self.timer=QtCore.QTimer(self);self.timer.timeout.connect(self.refresh);self.timer.start(100)

    def add(self):
        if self.table.rowCount()>=50:return
        i=self.table.rowCount();self.table.insertRow(i)
        for j,v in enumerate((self.goal.x.value(),self.goal.y.value(),3.)):
            self.table.setItem(i,j,QtWidgets.QTableWidgetItem(str(v)))
        self.table.selectRow(i)

    def remove(self):
        if self.table.currentRow()>=0:self.table.removeRow(self.table.currentRow())

    def move(self,offset):
        a=self.table.currentRow();b=a+offset
        if a<0 or not 0<=b<self.table.rowCount():return
        for j in range(3):
            first=self.table.takeItem(a,j);second=self.table.takeItem(b,j)
            self.table.setItem(a,j,second);self.table.setItem(b,j,first)
        self.table.selectRow(b)

    def points(self):
        try:return validate_route([dict(zip(('x','y','wait'),[self.table.item(i,j).text() for j in range(3)])) for i in range(self.table.rowCount())])
        except (AttributeError,ValueError) as exc:raise ValueError('地点の座標・待機時間を確認してください') from exc

    def write_route(self,path):
        if not self.goal.world:raise ValueError('コースを開始してください')
        data=dict(version=1,world=Path(self.goal.world).name,frame=self.goal.data.get('frame','world'),points=self.points())
        Path(path).write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')

    def read_route(self,path):
        data=json.loads(Path(path).read_text(encoding='utf-8'))
        if not isinstance(data,dict) or data.get('version')!=1:raise ValueError('ルート形式が不正です')
        if not self.goal.world or data.get('world')!=Path(self.goal.world).name:raise ValueError('このルートと同じコースを選択してください')
        if data.get('frame','world')!=self.goal.data.get('frame','world'):raise ValueError('座標系が異なるルートです')
        points=validate_route(data.get('points'))
        self.table.setRowCount(0)
        for point in points:
            i=self.table.rowCount();self.table.insertRow(i)
            for j,k in enumerate(('x','y','wait')):self.table.setItem(i,j,QtWidgets.QTableWidgetItem(str(point[k])))

    def save(self):
        folder=Path(__file__).resolve().parents[1]/'routes';folder.mkdir(exist_ok=True)
        path,_=QtWidgets.QFileDialog.getSaveFileName(self,'配送ルートを保存',str(folder/'route.json'),'JSON (*.json)')
        if path:
            try:self.write_route(path);self.status.setText('保存しました')
            except (OSError,ValueError) as exc:self.status.setText(str(exc))

    def load(self):
        folder=Path(__file__).resolve().parents[1]/'routes'
        path,_=QtWidgets.QFileDialog.getOpenFileName(self,'配送ルートを読込',str(folder),'JSON (*.json)')
        if path:
            try:self.read_route(path);self.status.setText('読み込みました（走行は開始しません）')
            except (OSError,ValueError) as exc:self.status.setText(str(exc))

    def abandon(self):self.pending=None

    def reset(self):
        self.abandon();self.table.setRowCount(0);self.status.setText('配送地点を登録してください')

    def send(self,op):
        if not self.goal.connected_node or time.monotonic()-self.goal.last_received>1:
            self.status.setText('制御に接続待ち');return
        try:points=self.points() if op=='start' else []
        except ValueError as exc:self.status.setText(str(exc));return
        self.goal.abandon_requests();self.goal.owner.clear_input()
        request=dict(id=str(uuid.uuid4()),op=op,world=self.goal.world,frame=self.goal.data.get('frame','world'),points=points)
        self.pending=request;self.sent=time.monotonic();self.pub.publish(String(data=json.dumps(request)))
        self.status.setText('応答待ち…')

    def refresh(self):
        data=self.goal.data;reply=data.get('mission_reply',{})
        if self.pending:
            if reply.get('id')==self.pending['id']:
                op=self.pending['op'];self.pending=None
                if not reply.get('ok'):
                    self.status.setText(reply.get('detail','要求に失敗しました'));return
                if op in ('start','resume'):
                    self.goal.owner.request(self.goal.node.mode_client,False,'配送ミッションを開始／再開しました。停止ロック中は解除してください。')
            elif time.monotonic()-self.sent>8:
                self.pending=None;self.status.setText('応答を確認できません。停止して接続を確認してください');return
            else:return
        mission=data.get('mission',{})
        if mission.get('state','idle')!='idle':
            text=mission.get('detail','')
            if mission.get('paused'):text='一時停止中 / '+text
            if mission.get('state')=='waiting':text+=f" 残り {mission.get('remaining',0):.1f} 秒"
            self.status.setText(text)
