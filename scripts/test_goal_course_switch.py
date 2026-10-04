"""目的地移動中にコースを切り替え、古い目的地・経路・入力が残らないことを実機物理で確認する。"""
import os
os.environ['QT_QPA_PLATFORM']='offscreen'
os.environ.setdefault('ROS_DOMAIN_ID','42')
import json
import time
from pathlib import Path
import rclpy
from PyQt5 import QtWidgets
from manual_panel import PanelNode,configure_japanese_font
from course_selector import CoursePanel

ROOT=Path(__file__).resolve().parents[1]
app=QtWidgets.QApplication([]);configure_japanese_font(app);rclpy.init()
node=PanelNode();panel=CoursePanel(node,simulation_gui=False);panel.show()


def until(test,seconds=40):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        app.processEvents()
        if test():return
        time.sleep(.02)
    raise TimeoutError(panel.navigation.data)


try:
    panel.combo.setCurrentIndex(3);panel.switch_course()
    until(lambda:panel.navigation.start.isEnabled() and node.mode=='manual' and bool(panel.navigation.data.get('pose')))
    panel.navigation.select(5.,3.);panel.navigation.go()
    until(lambda:node.mode=='auto' and panel.navigation.data.get('state')=='navigating' and abs(node.actual_speed)>.1)
    panel.combo.setCurrentIndex(2);panel.switch_course()
    until(lambda:panel.stopping_since is None and panel.navigation.data.get('world','').endswith('oval_course.sdf')
          and node.mode=='manual' and time.monotonic()-node.state_time<.5
          and bool(panel.navigation.data.get('pose')) and abs(node.actual_speed)<.02)
    data=panel.navigation.data
    assert data['state']=='idle' and data['goal'] is None and not data['path'],data
    assert abs(node.actual_speed)<.02,node.actual_speed
    (ROOT/'logs/goal_course_switch_validation.json').write_text(json.dumps({'passed':True,'new_course':data},indent=2)+'\n')
    panel.grab().save(str(ROOT/'logs/goal_course_switch.png'))
    print('PASS moving goal -> course switch -> manual stop with no old goal',flush=True)
finally:
    panel.close();until(lambda:panel.process is None,35)
    node.destroy_node();rclpy.try_shutdown()
