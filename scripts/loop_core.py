# ファイルの役割: ROSに依存しない距離・角度・速度制限の計算。手動と自動で同じ安全判定を使用する。
"""Pure geometry used by the simulated cart controller and safety gate."""
import math
from speed_settings import MAX_FORWARD_SPEED, MAX_REVERSE_SPEED, MAX_TURN_SPEED

ROBOT_BOUND = 0.62  # Includes all four tires (not just the chassis).


# 角度を±πの範囲へ戻し、角度が一周する境界でも差分を正しく扱う。
def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


# 有効な測距だけを取り出し、全方向・前方・後方の最短距離を返す。判定できなければNone。
def scan_distances(ranges, angle_min, angle_increment, range_min, range_max):
    valid = [(i, d) for i, d in enumerate(ranges)
             if math.isfinite(d) and range_min <= d <= range_max]
    front = [d for i, d in valid if abs(wrap(angle_min+i*angle_increment)) < math.radians(35)]
    rear = [d for i, d in valid if abs(wrap(angle_min+i*angle_increment)) > math.radians(145)]
    # Enclosed track must return actual walls in every direction.
    if len(valid) < len(ranges) * 0.5 or not front or not rear:
        return None
    return min(d for _, d in valid), min(front), min(rear)


# 進行方向の距離と制動距離に応じて並進・旋回を制限し、出力値と状態名を返す。
def safe_command(v, omega, nearest, front, rear=None):
    if v < 0 and rear is None:
        return 0.0, 0.0, 'invalid_scan_stop'
    # 前進なら前方、後退なら後方の距離を使う。距離は車体表面ではなくLiDAR中心からの値。
    travel_distance = rear if v < 0 else front
    if nearest <= 0.78 or (abs(v) > 1e-6 and travel_distance <= 0.95):
        return 0.0, 0.0, 'obstacle_stop'
    direction_scale = max(0.0, (travel_distance-0.95)/0.8) if abs(v) > 1e-6 else 1.0
    scale = min(1.0, direction_scale, max(0.0, (nearest-0.78)/0.4))
    limited_v = max(-MAX_REVERSE_SPEED, min(MAX_FORWARD_SPEED, v))
    # v*t + v^2/(2*a) must fit in the remaining clearance. Use conservative
    # braking vs the Gazebo drive limits and allow for observation/command delay.
    if abs(limited_v) > 1e-6:
        # 停止境界までの残り距離に、反応遅れと制動距離が収まる最大速度を計算する。
        available = max(0.0, travel_distance - .95)
        braking, reaction = 1.0, .35
        safe_speed = math.sqrt((braking*reaction)**2 + 2*braking*available) - braking*reaction
        scale = min(scale, safe_speed/abs(limited_v))
    return limited_v*scale, max(-MAX_TURN_SPEED, min(MAX_TURN_SPEED, omega))*scale, 'driving' if scale >= .999 else 'slowing'


# 従来の円形コースの壁箱との最短距離から車体を囲む半径を引く。任意コース用ではない。
def clearance_lower_bound(x, y):
    """Distance to every actual wall box minus a cart bounding circle."""
    best = math.inf
    for radius in (3.0, 6.0):
        half_len = radius * math.tan(math.pi/72) + .0125
        for i in range(72):
            a = i * math.tau/72
            radial = x*math.cos(a) + y*math.sin(a) - radius
            tangent = -x*math.sin(a) + y*math.cos(a)
            d = math.hypot(max(abs(radial)-.08, 0), max(abs(tangent)-half_len, 0))
            best = min(best, d)
    return best - ROBOT_BOUND
