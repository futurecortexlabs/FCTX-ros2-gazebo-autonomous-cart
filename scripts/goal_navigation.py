"""既知のSDF地図とGazebo位置を使う、A*経路計画と停止保持付き目的地走行。"""
import heapq
import math
import numpy as np
from loop_core import wrap


class NavigationMap:
    # 車体を囲む半径0.62mに余裕を加え、平面グリッドを作る。
    def __init__(self, geometry, resolution=.10):
        self.geometry = geometry
        self.dynamic = []
        self.resolution = resolution
        extents = []
        for x, y, a, hx, hy, kind in geometry.shapes:
            rx = hx if kind == 'circle' else abs(hx*math.cos(a))+abs(hy*math.sin(a))
            ry = hx if kind == 'circle' else abs(hx*math.sin(a))+abs(hy*math.cos(a))
            extents.append((x-rx, x+rx, y-ry, y+ry))
        if not extents:
            raise ValueError('コース形状を読み取れません')
        self.xmin = math.floor(min(e[0] for e in extents)-.3)
        self.xmax = math.ceil(max(e[1] for e in extents)+.3)
        self.ymin = math.floor(min(e[2] for e in extents)-.3)
        self.ymax = math.ceil(max(e[3] for e in extents)+.3)
        self.xs = np.arange(self.xmin, self.xmax+resolution/2, resolution)
        self.ys = np.arange(self.ymin, self.ymax+resolution/2, resolution)
        xx, yy = np.meshgrid(self.xs, self.ys)
        self._mesh=(xx,yy)
        self.clearances = self.distances(xx, yy)
        self.static_clearances = self.clearances.copy()
        self.free = self.clearances >= .28

    # 箱と円柱の距離をまとめて計算する。SDFから得た位置は計画に使用する。
    def distances(self, x, y):
        best = np.full(np.broadcast(x, y).shape, np.inf)
        for cx, cy, a, hx, hy, kind in self.geometry.shapes + self.dynamic:
            dx, dy = x-cx, y-cy
            if kind == 'circle':
                d = np.maximum(0., np.hypot(dx, dy)-hx)
            else:
                u, v = dx*math.cos(a)+dy*math.sin(a), -dx*math.sin(a)+dy*math.cos(a)
                d = np.hypot(np.maximum(0., np.abs(u)-hx), np.maximum(0., np.abs(v)-hy))
            best = np.minimum(best, d)
        return best-.62

    def update_obstacles(self, points):
        # 静的地図の距離は再利用し、LiDARで見つけた障害物だけを重ねる。
        self.dynamic=[(x,y,0.,.23,0.,'circle') for x,y in points]
        self.clearances=self.static_clearances.copy()
        if not hasattr(self,'_mesh'):self._mesh=np.meshgrid(self.xs,self.ys)
        xx,yy=self._mesh
        for x,y,_,radius,_,_ in self.dynamic:
            self.clearances=np.minimum(self.clearances,np.maximum(0.,np.hypot(xx-x,yy-y)-radius)-.62)
        self.free=self.clearances>=.28

    def cell(self, point):
        x, y = point
        return round((y-self.ymin)/self.resolution), round((x-self.xmin)/self.resolution)

    def point(self, cell):
        return float(self.xs[cell[1]]), float(self.ys[cell[0]])

    def valid(self, cell):
        return 0 <= cell[0] < len(self.ys) and 0 <= cell[1] < len(self.xs) and self.free[cell]

    def visible(self, a, b, margin=.28):
        n = max(2, math.ceil(math.dist(a, b)/.04)+1)
        t = np.linspace(0., 1., n)
        return bool(np.min(self.distances(a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1]))) >= margin)

    def plan(self, start, goal):
        # 壁際・障害物内の目的地は補正せず拒否する。指定した点への到着条件を曖昧にしない。
        if not all(math.isfinite(v) for v in (*start, *goal)):
            raise ValueError('目的地の座標が不正です')
        if not (self.xmin <= goal[0] <= self.xmax and self.ymin <= goal[1] <= self.ymax):
            raise ValueError('目的地が地図の範囲外です')
        if float(self.distances(*goal)) < .40:
            raise ValueError('目的地が壁・障害物に近すぎます')
        source, target = self.cell(start), self.cell(goal)
        if not self.valid(source) or not self.visible(start, self.point(source), .20):
            raise ValueError('現在位置に走行余裕がありません。手動で広い場所へ移動してください')
        if not self.valid(target) or not self.visible(self.point(target), goal):
            raise ValueError('目的地への安全な進入経路がありません')
        queue = [(0., source)]
        costs, parent = {source: 0.}, {}
        closed = set()
        directions = [(dy, dx) for dy in (-1, 0, 1) for dx in (-1, 0, 1) if dy or dx]
        while queue:
            _, current = heapq.heappop(queue)
            if current in closed:
                continue
            if current == target:
                break
            closed.add(current)
            for dy, dx in directions:
                nxt = current[0]+dy, current[1]+dx
                if not self.valid(nxt):
                    continue
                # 斜め移動で壁の角をすり抜けないよう、隣接する縦横セルも確認する。
                if dx and dy and (not self.valid((current[0]+dy, current[1])) or
                                  not self.valid((current[0], current[1]+dx))):
                    continue
                step = math.hypot(dx, dy)*self.resolution
                cost = costs[current]+step*(1.+.22/(self.clearances[nxt]+.15)**2)
                if cost < costs.get(nxt, math.inf):
                    costs[nxt], parent[nxt] = cost, current
                    heapq.heappush(queue, (cost+math.dist(nxt, target)*self.resolution, nxt))
        else:
            raise ValueError('目的地まで通れる経路がありません')
        cells = [target]
        while cells[-1] != source:
            cells.append(parent[cells[-1]])
        raw = [start]+[self.point(c) for c in reversed(cells)]+[goal]
        # 見通せる点同士を接続し、細かいグリッドの蛇行を除く。接続線の余裕も再確認する。
        path, i = [start], 0
        while i < len(raw)-1:
            j = len(raw)-1
            while j > i+1 and not self.visible(raw[i], raw[j], .32):
                j -= 1
            if not self.visible(raw[i], raw[j], .20):
                raise ValueError('安全な経路を構成できません')
            path.append(raw[j]); i = j
        return path


class GoalNavigator:
    # holdは「目的地モード」を表す。到着・取消・失敗後も勝手に自由走行へ戻さない。
    def __init__(self, geometry):
        self.map = NavigationMap(geometry) if geometry else None
        self.hold = False
        self.state, self.detail = 'idle', '目的地を指定してください'
        self.goal, self.path, self.index = None, [], 0
        self.distance = None
        self.progress_position = None
        self.progress_time = 0.
        self.progress_yaw = 0.

    def set_goal(self, pose, goal, now):
        self.hold = True
        self.goal, self.path, self.index = goal, [], 0
        self.distance = math.dist(pose[:2], goal) if pose else None
        try:
            if self.map is None:
                raise ValueError('コース情報がありません')
            if pose is None:
                raise ValueError('現在位置を受信していません')
            self.path = self.map.plan(pose[:2], goal)
        except ValueError as exc:
            self.state, self.detail = 'rejected', str(exc)
            return False
        self.index = 1
        self.state, self.detail = 'navigating', '目的地への経路を計画しました'
        self.progress_position, self.progress_time = pose[:2], now
        self.progress_yaw = pose[2]
        return True

    def cancel(self, free=False):
        self.hold = not free
        self.path, self.goal, self.distance = [], None, None
        self.state = 'idle' if free else 'cancelled'
        self.detail = '自由回避モード' if free else '目的地移動を取り消して停止しました'

    def command(self, pose, speed, now, paused=False):
        if not self.hold or not self.path or self.state in ('arrived', 'blocked', 'cancelled', 'rejected'):
            return 0., 0.
        self.distance = math.dist(pose[:2], self.goal)
        if paused or speed <= 0:
            self.state, self.detail = 'paused', '手動・停止ロック・速度ゼロのため待機中'
            self.progress_position, self.progress_time = pose[:2], now
            return 0., 0.
        if self.distance <= .18:
            self.state, self.detail = 'arrived', '目的地に到着しました（許容距離0.18m）'
            return 0., 0.
        while (self.index < len(self.path)-1 and math.dist(pose[:2], self.path[self.index]) < .45
               and self.map.visible(pose[:2], self.path[self.index+1], .28)):
            self.index += 1
        target = self.path[self.index]
        dx, dy = target[0]-pose[0], target[1]-pose[1]
        error = wrap(math.atan2(dy, dx)-pose[2])
        turn = max(-.65, min(.65, 1.8*error))
        # 旋回も含めて実際の進捗を監視する。止まったまま旋回指令だけを出し続けない。
        if math.dist(pose[:2], self.progress_position) > .10 or abs(wrap(pose[2]-self.progress_yaw)) > .10:
            self.progress_position, self.progress_time = pose[:2], now
            self.progress_yaw = pose[2]
        if now-self.progress_time > 12.:
            self.state, self.detail = 'blocked', '進めないため停止しました。障害物を確認し目的地を再指定してください'
            return 0., 0.
        # 曲がり角では進行方向を十分に合わせる。横滑りによる壁側へのふくらみを抑える。
        if abs(error) > .12:
            self.state, self.detail = 'navigating', '経路の方向へ旋回中'
            return 0., turn
        self.state, self.detail = 'navigating', '目的地へ移動中'
        return min(speed, .55, .8*math.hypot(dx, dy)) * max(.25, 1.-abs(error)/.16), turn
