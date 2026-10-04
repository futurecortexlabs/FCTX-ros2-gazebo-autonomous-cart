#!/usr/bin/env python3
# ファイルの役割: --guardでは最終指令の安全監視、それ以外では自由回避・既知地図による目的地移動と走行記録を実行する。
"""Free avoidance, known-map goal navigation, telemetry and independent safety gate."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid, Path as RosPath
from rclpy.qos import QoSProfile, DurabilityPolicy
from occupancy_navigation import OccupancyNavigationMap
from map_cache import occupancy_signature
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rcl_interfaces.msg import SetParametersResult
from speed_settings import MAX_AUTO_SPEED, DEFAULT_AUTO_SPEED
from std_srvs.srv import SetBool, Trigger
from ros_gz_interfaces.msg import Contacts

from loop_core import wrap, scan_distances, safe_command, clearance_lower_bound

from obstacle_avoidance import ObstacleAvoidance
from world_clearance import WorldClearance
from goal_navigation import GoalNavigator
from mission_control import Mission
from dynamic_obstacles import DynamicObstacles

PARTS = ('body', 'front_left', 'rear_left', 'front_right', 'rear_right')


# メッセージ時刻をナノ秒単位の整数に変換し、更新有無を比較できるようにする。
def stamp(msg):
    return msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec


# 最終指令を一元管理する安全監視ノード。
class SafetyGate(Node):
    # 手動・自動の入力、センサー、接触、サービスを接続し、実時間50ms周期の安全監視を登録する。
    def __init__(self, mode='auto', pose_source='simulation'):
        super().__init__('loop_safety_gate')
        self.pub = self.create_publisher(Twist, '/loop/cmd_vel', 10)
        self.status = self.create_publisher(String, '/loop/safety_status', 10)
        self.last = dict(scan=0.0, odom=0.0)
        self.mode = mode
        self.commands = {'auto': Twist(), 'manual': Twist()}
        self.command_times = {'auto': 0.0, 'manual': 0.0}
        self.switch_until = 0.0
        self.mode_pub = self.create_publisher(String, '/loop/control_mode', 10)
        self.stamps = dict(scan=None, odom=None)
        self.distance = None
        self.enabled = True
        self.collision = False
        self.state = ''
        self.create_subscription(Twist, '/loop/cmd_raw', lambda m: self.receive_command('auto', m), 1)
        self.create_subscription(Twist, '/loop/cmd_manual', lambda m: self.receive_command('manual', m), 1)
        self.create_service(SetBool, '/loop/set_manual', self.set_manual)
        self.create_subscription(LaserScan, '/loop/scan', self.scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/loop/ground_truth' if pose_source=='simulation' else '/loop/localized_odom', self.odom, qos_profile_sensor_data)
        for part in PARTS:
            self.create_subscription(Contacts, '/loop/contacts/'+part, self.contacts, qos_profile_sensor_data)
        self.create_service(SetBool, '/loop/set_enabled', self.enable)
        self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    # 時刻が前回と異なる場合だけ実時間の受信記録を更新し、同じデータの再送を正常更新と扱わない。
    def fresh_stamp(self, key, msg):
        value = stamp(msg)
        if value != self.stamps[key]:
            self.stamps[key] = value
            self.last[key] = time.monotonic()

    # 自動・手動それぞれの最新指令と受信時刻を保存し、選択中のモードだけを後で使用する。
    def receive_command(self, source, msg):
        self.commands[source] = msg
        self.command_times[source] = time.monotonic()

    # モードを変更し、古い指令を消して0.25秒の切替停止を設ける。
    def set_manual(self, req, res):
        self.mode = 'manual' if req.data else 'auto'
        self.commands = {'auto': Twist(), 'manual': Twist()}
        self.command_times = {'auto': 0.0, 'manual': 0.0}
        self.switch_until = time.monotonic() + .25
        self.pub.publish(Twist())
        self.mode_pub.publish(String(data=self.mode))
        self.get_logger().info('Control mode: '+self.mode)
        res.success = True
        res.message = self.mode
        return res

    # 測距の更新時刻と、前後・全方向の最短距離を安全監視用に保存する。
    def scan(self, msg):
        self.fresh_stamp('scan', msg)
        self.distance = scan_distances(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max)

    # 位置メッセージの更新有無を記録する。位置そのものから操舵を決める処理ではない。
    def odom(self, msg):
        self.fresh_stamp('odom', msg)

    # 物体名にwallまたはobstacle_がある接触を見つけたら、再起動まで残る停止フラグを立てる。
    def contacts(self, msg):
        if any(any(k in c.collision1.name+' '+c.collision2.name for k in ('wall', 'obstacle_')) for c in msg.contacts):
            self.collision = True

    # 停止ロックの設定・解除要求を処理する。接触フラグは消さず、接触後の解除は失敗として返す。
    def enable(self, req, res):
        self.enabled = req.data
        self.commands['manual'] = Twist()
        self.command_times['manual'] = 0.0
        # 停止条件を先に評価する。条件に該当した場合、初期値のゼロ指令をそのまま送る。
        if not self.enabled:
            self.pub.publish(Twist())
        res.success = not (req.data and self.collision)
        res.message = 'Wall or obstacle contact latched; restart simulation.' if not res.success else ('Enabled' if req.data else 'Stopped')
        return res

    # 停止要求、接触、切替、通信鮮度、入力異常を優先順に判定し、安全制限後の最終速度だけを配信する。
    def tick(self):
        output = Twist()
        now = time.monotonic()
        requested = self.commands[self.mode]
        command_timeout = .35
        # 停止条件を先に評価する。条件に該当した場合、初期値のゼロ指令をそのまま送る。
        if not self.enabled:
            state = 'user_stop'
        elif self.collision:
            state = 'wall_contact_stop'
        elif now < self.switch_until:
            state = 'mode_switch_stop'
        elif now - self.command_times[self.mode] > command_timeout:
            state = 'manual_input_timeout' if self.mode == 'manual' else 'waiting_for_fresh_data'
        elif any(now - value > .25 for value in self.last.values()):
            state = 'waiting_for_fresh_data'
        elif self.distance is None:
            state = 'invalid_scan_stop'
        elif not all(math.isfinite(v) for v in (requested.linear.x, requested.angular.z)):
            state = 'invalid_command_stop'
        else:
            output.linear.x, output.angular.z, state = safe_command(
                requested.linear.x, requested.angular.z, *self.distance)
        if state == 'driving' and abs(output.linear.x) + abs(output.angular.z) < 1e-6:
            state = 'stopped'
        self.pub.publish(output)
        self.status.publish(String(data=state))
        self.mode_pub.publish(String(data=self.mode))
        if state != self.state:
            self.get_logger().info('Safety: '+state)
            self.state = state


# 自動回避の候補速度と走行記録を担当するノード。
class AutonomousDriver(Node):
    # 回避器、速度パラメーター、記録用の形状、購読・配信先を準備し、自動判断を50ms周期で登録する。
    def __init__(self, laps, report, auto_speed=DEFAULT_AUTO_SPEED, world=None, pose_source='simulation'):
        super().__init__('loop_autonomous_driver')
        self.declare_parameter('max_speed', float(auto_speed))
        self.add_on_set_parameters_callback(self.validate_speed)
        self.world = world
        self.pose_source=pose_source
        self._map_signature=None
        self.frame='world' if pose_source=='simulation' else 'map'
        self.geometry = WorldClearance(world) if world and pose_source=='simulation' else None
        self.wall_geometry = WorldClearance(world, walls_only=True) if world and pose_source=='simulation' else None
        self.min_environment_clearance = math.inf
        self.goal_laps = laps
        self.planner = ObstacleAvoidance()
        self.scan_data = None
        self.scan_time = 0.0
        self.scan_stamp = None
        self.report = Path(report)
        self.report.parent.mkdir(parents=True, exist_ok=True)
        self.pub = self.create_publisher(Twist, '/loop/cmd_raw', 10)
        self.path_pub = self.create_publisher(RosPath, '/loop/path', 10)
        self.path = RosPath()
        self.path.header.frame_id = self.frame
        self.pose = None
        self.actual_velocity = (0., 0.)
        self.selected_velocity = (0., 0.)
        self.pose_time = 0.0
        self.pose_stamp = None
        self.navigator = GoalNavigator(self.geometry)
        self.dynamic_obstacles = DynamicObstacles(self.geometry) if self.geometry else None
        self.last_obstacle_update = 0.
        self.last_replan = 0.
        self.replan_count = 0
        self.obstacle_overflow = False
        self.mission = Mission(self.navigator)
        self.mission_reply = {}
        self.create_subscription(String, '/loop/mission_command', self.mission_command, 10)
        self.nav_pub = self.create_publisher(String, '/loop/navigation_status', 1)
        self.goal_path_pub = self.create_publisher(RosPath, '/loop/planned_path', 1)
        self.create_subscription(PoseStamped, '/loop/goal', self.receive_goal, 1)
        self.create_service(Trigger, '/loop/cancel_goal', self.cancel_goal)
        self.create_service(Trigger, '/loop/free_drive', self.free_drive)
        if pose_source!='simulation':
            self.create_subscription(OccupancyGrid,'/map',self.receive_map,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.last_nav_publish = 0.
        self.last_angle = None
        self.progress = 0.0
        self.first_stamp = None
        self.elapsed = 0.0
        self.samples = 0
        self.min_clearance = math.inf
        self.min_scan = math.inf
        # 半径4.5mの円形コース用の旧指標。楕円や倉庫では経路誤差として解釈しない。
        self.max_error = 0.0
        self.contacts_count = 0
        self.obstacle_contacts_count = 0
        self.floor_messages = 0
        self.contact_parts = set()
        self.scan_count = 0
        self.completed = False
        self.completion_sim_seconds = None
        self.last_log = 0.0
        self.safety = 'initializing'
        self.mode = 'auto'
        self.create_subscription(String, '/loop/control_mode', self.control_mode, 10)
        self.create_subscription(Odometry, '/loop/ground_truth' if pose_source=='simulation' else '/loop/localized_odom', self.odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/loop/scan', self.scan, qos_profile_sensor_data)
        self.create_subscription(String, '/loop/safety_status', lambda m: setattr(self, 'safety', m.data), 10)
        for part in PARTS:
            self.create_subscription(Contacts, '/loop/contacts/'+part, lambda msg, p=part: self.contacts(msg, p), qos_profile_sensor_data)
        self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    # 自動速度の変更値が有限で、許可範囲0～1.50m/sに収まっているか検証する。
    def validate_speed(self, parameters):
        for p in parameters:
            if p.name == 'max_speed':
                if not isinstance(p.value, (float, int)) or not math.isfinite(p.value) or not 0.0 <= p.value <= MAX_AUTO_SPEED:
                    return SetParametersResult(successful=False, reason=f'max_speed must be 0.0..{MAX_AUTO_SPEED} m/s')
        return SetParametersResult(successful=True)

    def receive_map(self,msg):
        if msg.header.frame_id!='map' or msg.info.width<2:return
        signature=occupancy_signature(msg)
        if signature==self._map_signature:return
        try:new_map=OccupancyNavigationMap(msg)
        except ValueError as exc:self.get_logger().error(str(exc));return
        if self.dynamic_obstacles:
            # SLAMが箱や壁を地図へ取り込んだら、同じ点の動的半径を重ねない。
            # 未観測の点を時間だけで消すことはせず、新地図の占有観測で照合する。
            self.dynamic_obstacles.replace_geometry(new_map.geometry)
            new_map.update_obstacles(self.dynamic_obstacles.points)
        self._map_signature=signature
        self.navigator.map=new_map;self.geometry=new_map.geometry
        if self.dynamic_obstacles is None:self.dynamic_obstacles=DynamicObstacles(self.geometry)
        self.obstacle_overflow=len(self.dynamic_obstacles.cells)>500

    def mission_command(self, msg):
        # 応答IDを戻し、古い応答からGUIが自動走行を開始しないようにする。
        request={};ok=False;detail=''
        try:
            request=json.loads(msg.data)
            if not isinstance(request,dict):raise ValueError('要求形式が不正です')
            op=request.get('op')
            if op=='start':
                if request.get('world') != self.world or request.get('frame','world')!=self.frame:raise ValueError('保存ルートと現在のコースが異なります')
                pose=self.pose if time.monotonic()-self.pose_time<.5 else None
                ok=self.mission.start(request.get('points'),pose,time.monotonic());detail=self.mission.detail
            elif op in ('pause','resume'):
                if not self.mission.active:raise ValueError('実行中の配送がありません')
                self.mission.paused=op=='pause';ok=True
            elif op=='cancel':self.mission.cancel();ok=True
            else:raise ValueError('未対応の配送操作です')
        except (ValueError,TypeError) as exc:detail=str(exc)
        self.pub.publish(Twist())
        self.mission_reply=dict(id=request.get('id') if isinstance(request,dict) else None,ok=ok,detail=detail)
        self.publish_goal_path();self.publish_navigation()

    # 目的地はworld座標で受け付ける。位置が古い場合や不正な座標は停止保持のまま拒否する。
    def receive_goal(self, msg):
        if self.mission.active:self.mission.cancel()
        goal = (msg.pose.position.x, msg.pose.position.y)
        self.pub.publish(Twist())
        if msg.header.frame_id != self.frame or not all(math.isfinite(v) for v in goal):
            self.navigator.cancel()
            self.navigator.state, self.navigator.detail = 'rejected', self.frame+'座標の有限な目的地を指定してください'
        else:
            pose = self.pose if time.monotonic()-self.pose_time < .5 else None
            self.navigator.set_goal(pose, goal, time.monotonic())
        self.publish_goal_path()
        self.publish_navigation()

    # 取消後に自由走行へ戻さず停止を保持する。
    def cancel_goal(self, req, res):
        if self.mission.active:self.mission.cancel()
        self.navigator.cancel()
        self.pub.publish(Twist())
        self.publish_goal_path()
        self.publish_navigation()
        res.success, res.message = True, '目的地を取り消しました'
        return res

    # 自由回避への復帰は明示的な操作でだけ許可する。
    def free_drive(self, req, res):
        if self.mission.active:self.mission.cancel()
        self.navigator.cancel(free=True)
        self.planner.reset()
        self.pub.publish(Twist())
        self.publish_goal_path()
        self.publish_navigation()
        res.success, res.message = True, '自由回避モード'
        return res

    def publish_goal_path(self):
        path = RosPath()
        path.header.frame_id = self.frame
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.navigator.path:
            pose = PoseStamped(header=path.header)
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.w = 1.
            path.poses.append(pose)
        self.goal_path_pub.publish(path)

    def publish_navigation(self):
        nav = self.navigator
        bounds = [nav.map.xmin, nav.map.xmax, nav.map.ymin, nav.map.ymax] if nav.map else [-9,9,-7,7]
        self.nav_pub.publish(String(data=json.dumps(dict(
            world=self.world, frame=self.frame, pose_source=self.pose_source, bounds=bounds, state=nav.state, detail=nav.detail,
            goal=nav.goal, path=nav.path, pose=self.pose, distance=nav.distance,
            active=nav.hold, waypoint_index=nav.index, mission=self.mission.status(), mission_reply=self.mission_reply, dynamic_obstacles=self.dynamic_obstacles.points if self.dynamic_obstacles else [], replan_count=self.replan_count), ensure_ascii=False)))
        self.last_nav_publish = time.monotonic()

    # モードが変わったら旋回履歴を消し、新しいモードを保存する。
    def control_mode(self, msg):
        if msg.data != self.mode:
            if msg.data == 'manual' and self.mission.active:self.mission.cancel()
            self.planner.reset()
            if msg.data == 'manual' and self.navigator.hold and self.navigator.state in ('navigating', 'paused'):
                self.navigator.cancel()
                self.publish_goal_path()
        self.mode = msg.data

    # 壁・障害物・床の接触要素を別々に集計する。独立した衝突事故の回数とは限らない。
    def contacts(self, msg, part):
        self.contact_parts.add(part)
        for c in msg.contacts:
            names = c.collision1.name+' '+c.collision2.name
            if 'wall' in names:
                self.contacts_count += 1
            elif 'obstacle_' in names:
                self.obstacle_contacts_count += 1
            elif 'ground' in names:
                self.floor_messages += 1

    # 最新の測距と鮮度を保存し、試験記録用の測定数・最短距離を更新する。
    def scan(self, msg):
        if stamp(msg) != self.scan_stamp:
            self.scan_stamp = stamp(msg)
            self.scan_time = time.monotonic()
        self.scan_data = msg
        distances = scan_distances(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max)
        if distances:
            self.scan_count += 1
            self.min_scan = min(self.min_scan, distances[0])

    # 位置・姿勢・実速度、正味周回角、幾何距離、表示用軌跡を更新する。自由回避器には位置を渡さず、目的地移動では位置を使用する。
    def odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.pose = (p.x, p.y, yaw)
        self.actual_velocity = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        if stamp(msg) != self.pose_stamp:
            self.pose_stamp = stamp(msg)
            self.pose_time = time.monotonic()
        a = math.atan2(p.y, p.x)
        if self.last_angle is not None:
            self.progress += wrap(a-self.last_angle)
        self.last_angle = a
        t = stamp(msg) / 1e9
        if self.first_stamp is None:
            self.first_stamp = t
        self.elapsed = t-self.first_stamp
        self.samples += 1
        self.min_clearance = min(self.min_clearance, self.wall_geometry.clearance(p.x, p.y) if self.wall_geometry else (self.geometry.clearance(p.x,p.y) if self.geometry else math.inf))
        if self.geometry:
            self.min_environment_clearance = min(self.min_environment_clearance, self.geometry.clearance(p.x, p.y))
        # 半径4.5mの円形コース用の旧指標。楕円や倉庫では経路誤差として解釈しない。
        self.max_error = max(self.max_error, abs(math.hypot(p.x, p.y)-4.5))
        if self.samples % 6 == 0:
            ps = PoseStamped(header=msg.header, pose=msg.pose.pose)
            self.path.poses.append(ps)
            self.path.poses = self.path.poses[-3000:]
            self.path.header.stamp = msg.header.stamp
            self.path_pub.publish(self.path)

    # 現在の走行状態をJSONへ書く。一時ファイルを完成させてから置換し、書きかけの読み出しを避ける。
    def write_report(self):
        # 有限な測定値は小数を丸め、未計測の無限大などはJSONのnullに対応するNoneへ変換する。
        def finite(v):
            return round(v, 5) if math.isfinite(v) else None
        data = dict(completed=self.completed, target_laps=self.goal_laps,
                    laps=round(self.progress/math.tau, 5), sim_seconds=round(self.elapsed, 3),
                    completion_sim_seconds=self.completion_sim_seconds,
                    course_world=self.world,
                    min_body_environment_clearance_lower_bound_m=finite(self.min_environment_clearance),
                    wall_contact_count=self.contacts_count,
                    obstacle_contact_count=self.obstacle_contacts_count,
                    collision_count=self.contacts_count+self.obstacle_contacts_count,
                    floor_contact_count=self.floor_messages,
                    contact_topics_received=sorted(self.contact_parts),
                    min_body_wall_clearance_lower_bound_m=finite(self.min_clearance),
                    min_lidar_distance_m=finite(self.min_scan), max_radial_error_m=finite(self.max_error),
                    odometry_samples=self.samples, valid_scan_messages=self.scan_count,
                    safety_state=self.safety, control_mode=self.mode, position=self.pose,
                    autonomy='Known-map goal navigation' if self.navigator.hold else 'Local LiDAR obstacle avoidance',
                    mission=self.mission.status(), replan_count=self.replan_count,
                    dynamic_obstacle_count=len(self.dynamic_obstacles.cells) if self.dynamic_obstacles else 0,
                    navigation_state=self.navigator.state, navigation_goal=self.navigator.goal,
                    goal_distance_m=self.navigator.distance, navigation_detail=self.navigator.detail,
                    planner_state=self.planner.reason,
                    auto_speed_limit_mps=float(self.get_parameter("max_speed").value),
                    selected_velocity=self.selected_velocity, actual_velocity=self.actual_velocity,
                    localization=self.pose_source, coordinate_frame=self.frame)
        # 一時ファイルが完成してから置換し、別プロセスが不完全なJSONを読みにくくする。
        temporary = self.report.with_suffix('.tmp')
        temporary.write_text(json.dumps(data, indent=2)+'\n')
        temporary.replace(self.report)

    def update_dynamic_route(self):
        now=time.monotonic();nav=self.navigator;scan=self.scan_data
        if (nav.map is None or self.dynamic_obstacles is None or scan is None or self.pose is None
                or now-self.pose_time>.25 or now-self.scan_time>.25
                or abs(stamp(scan)-(self.pose_stamp or 0))>150000000):return
        if now-self.last_obstacle_update>=.5:
            self.last_obstacle_update=now
            changed=self.dynamic_obstacles.update(self.pose,scan.ranges,scan.angle_min,scan.angle_increment,scan.range_min,scan.range_max)
            self.obstacle_overflow=len(self.dynamic_obstacles.cells)>500
            if changed and not self.obstacle_overflow:nav.map.update_obstacles(self.dynamic_obstacles.points)
        if not nav.hold or nav.goal is None or nav.state in ('arrived','cancelled','rejected'):return
        if self.mode!='auto' or self.safety in ('user_stop','wall_contact_stop') or self.mission.paused:return
        if self.obstacle_overflow:
            nav.state,nav.detail='waiting_obstacle','障害物情報が多すぎるため停止中。コースを再起動してください'
            nav.path=[];return
        points=[self.pose[:2]]+nav.path[nav.index:]
        obstructed=(bool(nav.map.dynamic) or self.pose_source!='simulation') and any(not nav.map.visible(a,b,.20) for a,b in zip(points,points[1:]))
        if not obstructed and nav.state!='waiting_obstacle':return
        # 新経路を作る前にゼロ指令。失敗時も自由走行へ戻さない。
        self.pub.publish(Twist())
        nav.path=[];nav.state='waiting_obstacle';nav.detail='障害物を検知し、迂回経路を確認しています'
        if now-self.last_replan<2.:return
        self.last_replan=now;goal=nav.goal
        if nav.set_goal(self.pose,goal,now):
            self.replan_count+=1;nav.detail='障害物を避ける経路へ更新しました'
        else:
            nav.state='waiting_obstacle';nav.detail='迂回できないため停止中。通路が空くと再確認します'
        self.publish_goal_path();self.publish_navigation()

    # 自動モードかつ新鮮な測距があるときだけ候補速度を計算し、完了条件と定期レポートも処理する。
    def tick(self):
        msg = Twist()
        # 原点周りの正味角度で完了を判定する。走行距離や倉庫の到着判定ではない。
        if not self.completed and self.goal_laps > 0 and abs(self.progress) >= self.goal_laps*math.tau:
            self.completed = True
            self.completion_sim_seconds = round(self.elapsed, 3)
        self.update_dynamic_route()
        mission_enabled=(self.mode=='auto' and self.safety not in ('user_stop','wall_contact_stop')
                         and self.pose is not None and time.monotonic()-self.pose_time<.5
                         and time.monotonic()-self.scan_time<.5 and float(self.get_parameter('max_speed').value)>0)
        previous_index=self.mission.index
        self.mission.tick(self.pose,time.monotonic(),mission_enabled)
        if previous_index!=self.mission.index:self.publish_goal_path()
        if self.navigator.hold:
            if self.pose is not None and time.monotonic()-self.pose_time < .5 and time.monotonic()-self.scan_time < .5:
                msg.linear.x, msg.angular.z = self.navigator.command(
                    self.pose, float(self.get_parameter('max_speed').value), time.monotonic(),
                    paused=(self.mission.active and self.mission.paused) or self.mode != 'auto' or self.safety in ('user_stop', 'wall_contact_stop'))
            elif self.navigator.state in ('navigating', 'paused'):
                self.navigator.state, self.navigator.detail = 'paused', '新しいセンサー・位置情報を待っています'
                self.navigator.progress_time = time.monotonic()
        elif self.mode == 'auto' and self.scan_data is not None and time.monotonic()-self.scan_time < .5 and not self.completed:
            scan = self.scan_data
            msg.linear.x, msg.angular.z = self.planner.command(
                scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max,
                max_speed=float(self.get_parameter("max_speed").value))
        if time.monotonic()-self.last_nav_publish > .2:
            self.publish_navigation()
        self.selected_velocity = (msg.linear.x, msg.angular.z)
        self.pub.publish(msg)
        if time.monotonic()-self.last_log > 3:
            self.write_report()
            self.get_logger().info(f'laps={self.progress/math.tau:.2f} clearance>={self.min_clearance:.3f}m contacts={self.contacts_count} {self.safety}' + (' COMPLETE' if self.completed else ''))
            self.last_log = time.monotonic()


# --guardでは最終指令の安全監視、それ以外では自由回避・既知地図による目的地移動と走行記録を実行する。 起動から終了処理までをまとめる入口。
def main():
    import signal
    parser = argparse.ArgumentParser()
    parser.add_argument('--world', default=None)
    parser.add_argument('--pose-source', choices=['simulation','slam','localization'], default='simulation')
    parser.add_argument('--guard', action='store_true')
    parser.add_argument('--mode', choices=['auto', 'manual'], default='auto')
    parser.add_argument('--laps', type=int, default=0)
    parser.add_argument('--auto-speed', type=float, default=DEFAULT_AUTO_SPEED)
    parser.add_argument('--report', default=str(Path(__file__).resolve().parents[1]/'logs/loop_status.json'))
    args, ros_args = parser.parse_known_args()
    if not math.isfinite(args.auto_speed) or not 0.0 <= args.auto_speed <= MAX_AUTO_SPEED:
        parser.error('auto-speed must be 0.0..1.5 m/s')
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = SafetyGate(args.mode,args.pose_source) if args.guard else AutonomousDriver(args.laps, args.report, args.auto_speed, args.world,args.pose_source)
    stopping=[False]
    def request_stop(*_):stopping[0]=True
    signal.signal(signal.SIGINT,request_stop);signal.signal(signal.SIGTERM,request_stop)
    try:
        while rclpy.ok() and not stopping[0]:rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(6):
            node.pub.publish(Twist())
            time.sleep(.03)
        if isinstance(node, AutonomousDriver):
            node.write_report()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
