"""Qt地図クリックからROSの目的地受理・取消までを専用ドメインで検証する。"""
import os
os.environ['QT_QPA_PLATFORM']='offscreen'
os.environ['ROS_DOMAIN_ID']='49'
import math
import json
from std_msgs.msg import String
import time
from pathlib import Path
import rclpy
from PyQt5 import QtCore,QtWidgets,QtTest
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from loop_controller import AutonomousDriver,SafetyGate
from manual_panel import PanelNode,configure_japanese_font
from course_selector import CoursePanel

ROOT=Path(__file__).resolve().parents[1]
app=QtWidgets.QApplication([]);configure_japanese_font(app);rclpy.init()
driver=AutonomousDriver(0,ROOT/'logs/goal_ui_test.json',world=str(ROOT/'worlds/warehouse_course.sdf'))
gate=SafetyGate('manual');node=PanelNode();panel=CoursePanel(node,simulation_gui=False)
panel.show();sequence=0


def until(test,seconds=8):
    global sequence
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        sequence+=1
        odom=Odometry();odom.header.stamp.sec=sequence;odom.pose.pose.position.y=-3.4;odom.pose.pose.orientation.w=1.
        scan=LaserScan();scan.header.stamp.sec=sequence;scan.angle_min=-math.pi;scan.angle_increment=math.tau/359;scan.range_min=.06;scan.range_max=15.;scan.ranges=[5.]*360
        driver.odom(odom);driver.scan(scan);gate.odom(odom);gate.scan(scan)
        for _ in range(5):
            rclpy.spin_once(driver,timeout_sec=0);rclpy.spin_once(gate,timeout_sec=0)
        app.processEvents()
        if test():return
        time.sleep(.02)
    raise TimeoutError(panel.navigation.data)


try:
    ui=panel.navigation
    until(lambda:ui.start.isEnabled() and bool(ui.map.shapes))
    # 地図の原点をクリックして座標へ反映し、開始ボタンから目的地を送信する。
    q=ui.map.point(0.,0.)
    QtTest.QTest.mouseClick(ui.map,QtCore.Qt.LeftButton,pos=q.toPoint())
    assert abs(ui.x.value())<.04 and abs(ui.y.value())<.04
    QtTest.QTest.mouseClick(ui.start,QtCore.Qt.LeftButton)
    until(lambda:driver.navigator.state=='navigating' and gate.mode=='auto')
    assert driver.navigator.hold
    print('PASS map click -> goal -> automatic mode',flush=True)
    ui.cancel();until(lambda:driver.navigator.state=='cancelled')
    driver.tick();assert driver.selected_velocity==(0.,0.)
    print('PASS UI cancel holds zero command',flush=True)
    ui.select(2.5,1.4);ui.go();until(lambda:driver.navigator.state=='rejected')
    driver.tick();assert driver.selected_velocity==(0.,0.)
    print('PASS obstacle goal rejected',flush=True)
    until(lambda:ui.data.get('state')=='rejected')
    panel.grab().save(str(ROOT/'logs/goal_panel_test.png'))
    data=dict(ui.data);data['bounds']=[-3.,24.,-3.,18.]
    ui.receive(String(data=json.dumps(data)));ui.select(23.,17.)
    assert ui.x.value()==23. and ui.y.value()==17.
    print('PASS destination beyond 20m map coordinate',flush=True)
finally:
    panel.close();node.destroy_node();gate.destroy_node();driver.destroy_node();rclpy.try_shutdown()
