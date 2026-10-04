#!/usr/bin/env python3
# ファイルの役割: 専用ROSドメイン43で疑似センサーを配信し、通信断・古い時刻・停止要求・接触時の最終指令を検証する。
"""Exercise the real ROS safety node in isolated domain 43, without Gazebo."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

os.environ['ROS_DOMAIN_ID'] = '43'
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ros_gz_interfaces.msg import Contacts, Contact
from std_srvs.srv import SetBool


# 専用ROSドメイン43で疑似センサーを配信し、通信断・古い時刻・停止要求・接触時の最終指令を検証する。 起動から終了処理までをまとめる入口。
def main():
    root = Path(__file__).resolve().parents[1]
    logfile = open(root/'logs/safety_test_node.log', 'w')
    proc = subprocess.Popen(['/usr/bin/python3', str(root/'scripts/loop_controller.py'), '--guard'], stdout=logfile, stderr=subprocess.STDOUT)
    rclpy.init()
    node = rclpy.create_node('safety_test_driver')
    pubs = {
        'scan': node.create_publisher(LaserScan, '/loop/scan', 10),
        'odom': node.create_publisher(Odometry, '/loop/ground_truth', 10),
        'cmd': node.create_publisher(Twist, '/loop/cmd_raw', 10),
        'contact': node.create_publisher(Contacts, '/loop/contacts/body', 10),
    }
    samples = []
    node.create_subscription(Twist, '/loop/cmd_vel', lambda m: samples.append((time.monotonic(), m.linear.x, m.angular.z)), 10)
    service = node.create_client(SetBool, '/loop/set_enabled')
    passed = []
    sequence = 1

    # 疑似センサーと指令を一定時間配信し、ゲートが最近返した速度が期待する停止・走行状態か確認する。
    def exercise(name, duration=1.25, *, scan=True, cmd=True, distance=2.0, repeat_stamp=False, moving=False):
        nonlocal sequence
        samples.clear()
        started = time.monotonic()
        while time.monotonic()-started < duration:
            if not repeat_stamp:
                sequence += 1
            laser = LaserScan()
            laser.header.stamp.sec = sequence
            laser.angle_min, laser.angle_increment = -math.pi, math.tau/719
            laser.range_min, laser.range_max = .06, 15.
            laser.ranges = [distance]*720
            odom = Odometry()
            odom.header.stamp.sec = int(time.monotonic()*1000)
            odom.pose.pose.orientation.w = 1.0
            requested = Twist()
            requested.linear.x = .4
            requested.angular.z = .1
            pubs['odom'].publish(odom)
            if scan:
                pubs['scan'].publish(laser)
            if cmd:
                pubs['cmd'].publish(requested)
            rclpy.spin_once(node, timeout_sec=.025)
            time.sleep(.015)
        recent = [s for s in samples if s[0] > time.monotonic()-.25]
        assert recent, name+': no output from gate'
        if moving:
            assert all(s[1] > .2 for s in recent), (name, recent)
        else:
            assert all(abs(s[1])+abs(s[2]) < 1e-6 for s in recent), (name, recent)
        passed.append(name)
        print('PASS', name, flush=True)

    # 停止・解除サービスへ要求を送り、応答が成功することをテスト側で確認する。
    def enable(value):
        assert service.wait_for_service(timeout_sec=3)
        future = service.call_async(SetBool.Request(data=value))
        rclpy.spin_until_future_complete(node, future, timeout_sec=3)
        assert future.result().success

    try:
        exercise('fresh_data_drives', duration=2.5, moving=True)
        exercise('near_wall_stops', distance=.7)
        exercise('fresh_data_recovers', moving=True)
        exercise('missing_scan_stops', scan=False)
        exercise('repeated_scan_timestamp_stops', repeat_stamp=True)
        exercise('fresh_data_recovers_again', moving=True)
        exercise('missing_controller_command_stops', cmd=False)
        exercise('invalid_scan_stops', distance=float('nan'))
        enable(False)
        exercise('manual_stop_service')
        enable(True)
        exercise('manual_resume_service', moving=True)
        c = Contact()
        c.collision1.name = 'loop_cart::base_link::body_collision'
        c.collision2.name = 'outer_wall::wall::wall_0'
        pubs['contact'].publish(Contacts(contacts=[c]))
        exercise('wall_contact_latches_stop')
        out = root/'logs/safety_validation.json'
        out.write_text(json.dumps({'passed': passed, 'all_passed': True}, indent=2)+'\n')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        proc.send_signal(signal.SIGINT)
        proc.wait(timeout=8)
        logfile.close()


if __name__ == '__main__':
    main()
