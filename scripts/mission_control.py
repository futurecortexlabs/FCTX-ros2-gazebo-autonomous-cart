"""配送地点の検証と、走行・待機・一時停止の状態管理。ROSやGUIに依存しない。"""
import math


def validate_route(points):
    if not isinstance(points,list) or not 1 <= len(points) <= 50:
        raise ValueError('地点は1〜50件で指定してください')
    result=[]
    for point in points:
        if not isinstance(point,dict):raise ValueError('地点の形式が不正です')
        try:x,y,wait=(float(point[k]) for k in ('x','y','wait'))
        except (KeyError,TypeError,ValueError):raise ValueError('座標と待機時間を確認してください')
        if not all(math.isfinite(v) for v in (x,y,wait)) or not 0 <= wait <= 3600:
            raise ValueError('座標は有限値、待機時間は0〜3600秒で指定してください')
        result.append(dict(x=x,y=y,wait=wait))
    return result


class Mission:
    def __init__(self, navigator):
        self.nav=navigator;self.points=[];self.index=0;self.state='idle'
        self.detail='配送地点を登録してください';self.remaining=0.;self.last=None;self.paused=False

    @property
    def active(self):return self.state in ('running','waiting')

    def cancel(self):
        self.state='cancelled';self.detail='配送を取り消しました';self.paused=False
        self.remaining=0.;self.nav.cancel()

    def start(self,points,pose,now):
        self.cancel()
        try:
            points=validate_route(points)
            if pose is None or self.nav.map is None:raise ValueError('現在位置・地図を確認してください')
            previous=pose[:2]
            # 出発前に全区間を検査し、到達できない地点を含む配送は開始しない。
            for point in points:
                goal=(point['x'],point['y']);self.nav.map.plan(previous,goal);previous=goal
            self.points=points;self.index=0;self.last=now
            self._target(pose,now)
        except ValueError as exc:
            self.state='failed';self.detail=str(exc);return False
        return self.active

    def _target(self,pose,now):
        point=self.points[self.index]
        if not self.nav.set_goal(pose,(point['x'],point['y']),now):
            self.state='failed';self.detail=self.nav.detail;return
        self.state='running';self.detail=f'地点 {self.index+1}/{len(self.points)} へ移動中'

    def tick(self,pose,now,enabled):
        dt=0. if self.last is None else max(0.,min(now-self.last,.25))
        self.last=now
        if not self.active:return
        if not enabled or self.paused:return
        if self.nav.state in ('blocked','rejected','cancelled'):
            self.state='failed';self.detail=self.nav.detail;return
        if self.state=='running' and self.nav.state=='arrived':
            self.state='waiting';self.remaining=self.points[self.index]['wait'];dt=0.
            self.detail=f'地点 {self.index+1}/{len(self.points)} で待機中'
        if self.state=='waiting':
            self.remaining=max(0.,self.remaining-dt)
            if self.remaining<=0:
                self.index+=1
                if self.index==len(self.points):
                    self.state='completed';self.detail='すべての配送地点を完了しました'
                else:self._target(pose,now)

    def status(self):
        return dict(state=self.state,detail=self.detail,paused=self.paused,index=self.index,
                    total=len(self.points),remaining=round(self.remaining,1),points=self.points)
