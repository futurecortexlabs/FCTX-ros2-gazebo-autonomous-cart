# ファイルの役割: LiDARから複数の進路候補を予測し、安全に前進できる候補を選ぶ。地図・目的地・固定周回方向は使わない。
"""Local LiDAR trajectory selection: no map, waypoints or prescribed lap direction."""
import math
import numpy as np
from speed_settings import MAX_AUTO_SPEED, DEFAULT_AUTO_SPEED


# LiDARの点群から将来の移動経路を評価し、安全な速度と旋回の組を選ぶ回避器。
class ObstacleAvoidance:
    # 前回の旋回、回復用の旋回方向、判断理由、予測用の時刻列を初期化する。
    def __init__(self):
        self.last_turn = 0.0
        self.recovery_turn = 0.0
        self.reason = 'waiting_for_scan'
        self.best_clearance = 0.0
        self.times = np.linspace(.15, 3.0, 20)

    # 前回の旋回指令と回復旋回方向を消し、前のモードの判断を持ち越さない。
    def reset(self):
        self.last_turn = self.recovery_turn = 0.0

    # 測距を確認し、前進速度と旋回速度の候補を予測・評価する。通過候補がなければ空いた側へその場旋回する。
    def command(self, ranges, angle_min, angle_increment, range_min, range_max, max_speed=DEFAULT_AUTO_SPEED):
        if not math.isfinite(max_speed) or max_speed <= 0:
            self.reason = "speed_limit_stop"
            return 0., 0.
        max_speed = min(max_speed, MAX_AUTO_SPEED)
        ranges = np.asarray(ranges, dtype=float)
        if ranges.size < 30 or not math.isfinite(angle_increment) or angle_increment <= 0:
            self.reason = 'invalid_scan'
            return 0., 0.
        # 各測定の角度を作る。以下では台車中心を原点とする平面上の障害物点へ変換する。
        angles = angle_min + np.arange(ranges.size)*angle_increment
        valid = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
        # Current lab is enclosed. Insufficient actual returns fail closed.
        if np.count_nonzero(valid) < ranges.size*.5:
            self.reason = 'invalid_scan'
            return 0., 0.
        d, a = ranges[valid], angles[valid]
        nearest = float(np.min(d))
        if nearest <= .79:
            self.reason = 'too_close_stop'
            return 0., 0.
        ox, oy = d*np.cos(a), d*np.sin(a)
        # Full robot circle (0.62m) plus prediction margin. The independent gate
        # still checks fresh scans and actual proximity on every output.
        # 車体を囲む半径0.62mより大きい中心間隔を求める。近接時は現在の空間から離れる候補も許す。
        margin = min(.90, nearest - .025)
        best = None
        turns = np.unique(np.append(np.linspace(-.8, .8, 33), self.last_turn))
        # 前進速度ごとに旋回候補を試し、各候補の将来位置と障害物の距離を評価する。
        for v in sorted(set((min(.18, max_speed), .4*max_speed, .67*max_speed, max_speed))):
            times = self.times * (max(1.5, min(3.0, 1.8/v))/3.0)
            for w in turns:
                predicted_w = w * .70  # Four-wheel skid steering turns less than ideal kinematics.
                if abs(predicted_w) < 1e-6:
                    px, py = v*times, np.zeros_like(times)
                else:
                    px = v/predicted_w*np.sin(predicted_w*times)
                    py = v/predicted_w*(1-np.cos(predicted_w*times))
                distances = np.sqrt((px[:, None]-ox)**2 + (py[:, None]-oy)**2)
                path_clearances = np.min(distances, axis=1)
                clearance = float(np.min(path_clearances))
                if clearance < margin:
                    continue
                # Prefer forward progress and usable space; avoid unnecessary
                # turning or alternating left/right between scans.
                # 速度と空間の余裕を評価し、強い旋回や前回からの急な操舵変更を減点する。
                score = 2.3*v + .5*min(clearance, 1.4) + .25*px[-1]
                score += 1.2*float(np.mean(np.minimum(path_clearances, 1.4)))
                score += .8*min(float(path_clearances[-1]), 1.4)
                score -= .25*abs(w) + .18*abs(w-self.last_turn)
                if best is None or score > best[0]:
                    best = (score, v, float(w), clearance)
        if best is not None:
            _, v, w, clearance = best
            self.last_turn = w
            self.recovery_turn = 0.0
            self.best_clearance = clearance
            self.reason = 'clear_path' if abs(w)<.03 else 'avoiding_wall'
            return v, w
        # No safe forward arc: turn in place toward the more open side.
        # Keep that choice until a traversable forward arc is available.
        if self.recovery_turn == 0.0:
            left = d[(a>.2) & (a<2.2)]
            right = d[(a<-.2) & (a>-2.2)]
            if not len(left) or not len(right):
                self.reason = 'no_safe_direction'
                return 0., 0.
            left_space = float(np.mean(np.minimum(left, 4.0)))
            right_space = float(np.mean(np.minimum(right, 4.0)))
            self.recovery_turn = .5 if left_space >= right_space else -.5
        self.last_turn = self.recovery_turn
        self.reason = 'turning_toward_free_space'
        return 0., self.recovery_turn
