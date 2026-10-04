#!/usr/bin/env python3
# ファイルの役割: 起動済みGazeboの台車へ指令を送り、前後進・旋回・入力途絶後の停止を実際の位置変化で検証する。
"""Exercise manual commands against the running Gazebo cart (panel closed)."""
import json
import math
import os
from pathlib import Path
import time
os.environ.setdefault('ROS_DOMAIN_ID', '42')
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
from ros_gz_interfaces.msg import Contacts


# Gazeboの実移動を測りながら試験指令を送るノード。
class Probe(Node):
    # このクラスで使う状態・通信先・画面部品を初期化する。
    def __init__(self):
        super().__init__('manual_drive_validation')
        self.pose = None
        self.wall_contacts = 0
        self.pub = self.create_publisher(Twist, '/loop/cmd_manual', 1)
        self.mode = self.create_client(SetBool, '/loop/set_manual')
        self.enable = self.create_client(SetBool, '/loop/set_enabled')
        self.create_subscription(Odometry, '/loop/ground_truth', self.odom, qos_profile_sensor_data)
        for part in ('body', 'front_left', 'rear_left', 'front_right', 'rear_right'):
            self.create_subscription(Contacts, '/loop/contacts/'+part, self.contact, qos_profile_sensor_data)

    # 台車の位置とyawを保存し、前後移動量・旋回角・停止後のずれの測定に使う。
    def odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = (p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))

    # この旧手動試走では壁接触のみを数える。障害物を含む総合判定は別の試走で確認する。
    def contact(self, msg):
        self.wall_contacts += sum('wall' in c.collision1.name or 'wall' in c.collision2.name for c in msg.contacts)

    # ROSサービスの準備と非同期応答を待ち、要求の成功を確認する。
    def call(self, client, value):
        assert client.wait_for_service(timeout_sec=5)
        future = client.call_async(SetBool.Request(data=value))
        rclpy.spin_until_future_complete(self, future, timeout_sec=3)
        assert future.result().success

    # ROS受信を進めながら一定周期で速度を送り、指定時間後の位置を返す。配信停止による通信断も試せる。
    def pump(self, seconds, v=0., w=0., publish=True):
        end, next_send = time.monotonic()+seconds, 0.
        while time.monotonic()<end:
            if publish and time.monotonic()>next_send:
                msg = Twist()
                msg.linear.x, msg.angular.z = float(v), float(w)
                self.pub.publish(msg)
                next_send = time.monotonic()+.05
            rclpy.spin_once(self, timeout_sec=.01)
        assert self.pose is not None
        return self.pose


# 起動済みGazeboの台車へ指令を送り、前後進・旋回・入力途絶後の停止を実際の位置変化で検証する。 起動から終了処理までをまとめる入口。
def main():
    rclpy.init()
    node = Probe()
    results = {}
    try:
        node.call(node.mode, True)
        node.call(node.enable, True)
        before = node.pump(1)
        node.pump(1.5, .4)
        after = node.pump(.8)
        forward = (after[0]-before[0])*math.cos(before[2]) + (after[1]-before[1])*math.sin(before[2])
        assert forward > .25, forward
        results['forward_displacement_m'] = round(forward, 4)
        before = after
        node.pump(1.5, -.25)
        after = node.pump(.8)
        reverse = (after[0]-before[0])*math.cos(before[2]) + (after[1]-before[1])*math.sin(before[2])
        assert reverse < -.15, reverse
        results['reverse_displacement_m'] = round(reverse, 4)
        for name, omega in [('left', .6), ('right', -.6)]:
            before = node.pose
            node.pump(1.5, 0., omega)
            after = node.pump(.8)
            delta = math.atan2(math.sin(after[2]-before[2]), math.cos(after[2]-before[2]))
            assert delta*omega > .1, (name, delta)
            results[name+'_yaw_change_rad'] = round(delta, 4)
        node.pump(1., .3)
        settled = node.pump(1., publish=False)
        after = node.pump(.6, publish=False)
        drift = math.hypot(after[0]-settled[0], after[1]-settled[1])
        assert drift < .02, drift
        results['drift_after_input_timeout_m'] = round(drift, 6)
        node.call(node.mode, False)
        before = node.pump(.4)
        after = node.pump(4.)
        distance = math.hypot(after[0]-before[0], after[1]-before[1])
        assert distance > .8, distance
        results['auto_resume_displacement_m'] = round(distance, 4)
        node.call(node.mode, True)
        settled = node.pump(1.)
        after = node.pump(.6)
        drift = math.hypot(after[0]-settled[0], after[1]-settled[1])
        assert drift < .02, drift
        results['drift_after_manual_switch_m'] = round(drift, 6)
        assert node.wall_contacts == 0, node.wall_contacts
        results['wall_contacts'] = node.wall_contacts
        results['all_passed'] = True
        out = Path(__file__).resolve().parents[1]/'logs/manual_drive_validation.json'
        out.write_text(json.dumps(results, indent=2)+'\n')
        print(json.dumps(results, indent=2))
    finally:
        node.call(node.mode, True)
        node.pump(.3)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
