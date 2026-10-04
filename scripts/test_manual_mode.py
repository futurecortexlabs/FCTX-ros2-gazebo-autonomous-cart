#!/usr/bin/env python3
# ファイルの役割: 専用ROSドメイン44で、手動と自動の指令選択、後退時の障害物、切替・停止を検証する。
"""ROS integration test for mode arbitration and reverse obstacle handling."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

os.environ['ROS_DOMAIN_ID'] = '44'
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool


# 専用ROSドメイン44で、手動と自動の指令選択、後退時の障害物、切替・停止を検証する。 起動から終了処理までをまとめる入口。
def main():
    root = Path(__file__).resolve().parents[1]
    log = open(root/'logs/manual_test_node.log', 'w')
    proc = subprocess.Popen(['/usr/bin/python3', str(root/'scripts/loop_controller.py'), '--guard', '--mode', 'manual'], stdout=log, stderr=subprocess.STDOUT)
    rclpy.init()
    node = rclpy.create_node('manual_mode_test')
    auto = node.create_publisher(Twist, '/loop/cmd_raw', 1)
    manual = node.create_publisher(Twist, '/loop/cmd_manual', 1)
    scan = node.create_publisher(LaserScan, '/loop/scan', 1)
    odom = node.create_publisher(Odometry, '/loop/ground_truth', 1)
    mode = node.create_client(SetBool, '/loop/set_manual')
    enabled = node.create_client(SetBool, '/loop/set_enabled')
    samples = []
    node.create_subscription(Twist, '/loop/cmd_vel', lambda m: samples.append((time.monotonic(), m.linear.x, m.angular.z)), 10)
    passed = []

    # ROSサービスの準備と非同期応答を待ち、要求の成功を確認する。
    def call(client, value):
        assert client.wait_for_service(timeout_sec=5)
        future = client.call_async(SetBool.Request(data=value))
        rclpy.spin_until_future_complete(node, future, timeout_sec=3)
        assert future.result().success

    # 指定した入力条件を配信して最終速度を採取し、手動モードの選択と安全判定を検証する。
    def check(name, expected, manual_cmd=None, front=2., rear=2., duration=1.0):
        started = time.monotonic()
        samples.clear()
        while time.monotonic()-started < duration:
            laser = LaserScan()
            laser.header.stamp.sec = int(time.monotonic()*1000)
            laser.angle_min, laser.angle_increment = -math.pi, math.tau/719
            laser.range_min, laser.range_max = .06, 15.
            laser.ranges = [rear if abs(-math.pi+i*math.tau/719)>math.radians(145) else (front if abs(-math.pi+i*math.tau/719)<math.radians(35) else 2.) for i in range(720)]
            pose = Odometry()
            pose.header.stamp = laser.header.stamp
            pose.pose.pose.orientation.w = 1.
            a = Twist()
            a.linear.x = .6
            auto.publish(a)  # Competing autopilot runs throughout every test.
            scan.publish(laser)
            odom.publish(pose)
            if manual_cmd is not None:
                m = Twist()
                m.linear.x, m.angular.z = manual_cmd
                manual.publish(m)
            rclpy.spin_once(node, timeout_sec=.02)
            time.sleep(.02)
        recent = [(v, w) for t, v, w in samples if t>time.monotonic()-.2]
        assert recent, (name, 'no gate output')
        assert all(expected(v, w) for v, w in recent), (name, recent)
        passed.append(name)
        print('PASS', name, flush=True)

    zero = lambda v, w: abs(v)+abs(w)<1e-6
    try:
        check('manual_start_ignores_auto', zero, duration=2.5)
        check('manual_forward_wins_over_auto', lambda v,w: abs(v-.4)<1e-6, (.4, 0.))
        check('manual_reverse', lambda v,w: abs(v+.2)<1e-6, (-.2, 0.))
        check('rear_obstacle_stops_reverse', zero, (-.2, 0.), rear=.85)
        check('can_reverse_away_from_front_wall', lambda v,w: v<-.01, (-.2, 0.), front=.85)
        check('manual_left_turn', lambda v,w: abs(v)<1e-6 and w>.4, (0., .5))
        check('manual_right_turn', lambda v,w: abs(v)<1e-6 and w<-.4, (0., -.5))
        check('manual_deadman_no_auto_fallback', zero)
        call(mode, False)
        check('switch_to_auto_ignores_manual', lambda v,w: abs(v-.6)<1e-6, (-.2, 0.))
        call(mode, True)
        check('switch_to_manual_discards_old_input', zero)
        call(enabled, False)
        check('manual_stop_lock', zero, (.4, 0.))
        call(enabled, True)
        check('manual_resume', lambda v,w: abs(v-.4)<1e-6, (.4, 0.))
        (root/'logs/manual_validation.json').write_text(json.dumps({'all_passed': True, 'passed': passed}, indent=2)+'\n')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        proc.send_signal(signal.SIGINT)
        proc.wait(timeout=8)
        log.close()


if __name__ == '__main__':
    main()
