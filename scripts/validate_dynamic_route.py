"""走行中に箱を追加し、LiDAR再計画・接触ゼロ・到着・削除後消去を実走検証する。"""
import os
os.environ['QT_QPA_PLATFORM']='offscreen';os.environ.setdefault('ROS_DOMAIN_ID','42')
import json,time,math,sys
mission_test='--mission' in sys.argv
import rclpy
from PyQt5 import QtWidgets
from course_selector import CoursePanel,PanelNode,COURSES,ROOT,configure_japanese_font
from obstacle_control import add,remove_all
app=QtWidgets.QApplication([]);configure_japanese_font(app);rclpy.init()
node=PanelNode();panel=CoursePanel(node,simulation_gui=False);panel.show()
def until(test,seconds=40):
    end=time.monotonic()+seconds;last=0
    while time.monotonic()<end:
        app.processEvents()
        if test():return
        if time.monotonic()-last>10:
            d=panel.navigation.data
            print(d.get('state'),d.get('distance'),'replans',d.get('replan_count'),'points',len(d.get('dynamic_obstacles',[])),flush=True);last=time.monotonic()
        time.sleep(.02)
    raise TimeoutError(panel.navigation.data)
try:
    panel.combo.setCurrentIndex(next(i for i,c in enumerate(COURSES) if c[1]=='large_warehouse_course.sdf'));panel.switch_course()
    until(lambda:node.mode=='manual' and panel.navigation.start.isEnabled() and bool(panel.navigation.data.get('pose')))
    panel.navigation.select(-3.,-7.5)
    if mission_test:
        panel.navigation.mission.add();panel.navigation.mission.send('start')
    else:panel.navigation.go()
    until(lambda:node.mode=='auto' and abs(node.actual_speed)>.1)
    print('ADDED',add(-6.5,-7.5),flush=True)
    until(lambda:panel.navigation.data.get('replan_count',0)>0,25)
    panel.grab().save(str(ROOT/'logs/dynamic_replan.png'))
    until(lambda:panel.navigation.data.get('state')=='arrived',220)
    t=time.monotonic();until(lambda:time.monotonic()-t>3,5)
    if mission_test:until(lambda:panel.navigation.data.get('mission',{}).get('state')=='completed',10)
    report=json.loads((ROOT/'logs/loop_status.json').read_text())
    assert report['collision_count']==0,report
    assert abs(node.actual_speed)<.02
    assert math.dist(panel.navigation.data['pose'][:2],(-3.,-7.5))<.20
    print('PASS inserted obstacle -> replan -> arrival without contact',flush=True)
    print('REMOVED',remove_all(),flush=True)
    until(lambda:not panel.navigation.data.get('dynamic_obstacles'),20)
    (ROOT/('logs/dynamic_mission_validation.json' if mission_test else 'logs/dynamic_route_validation.json')).write_text(json.dumps(dict(passed=True,report=report),indent=2))
    print('PASS cleared obstacle observations after removal',flush=True)
finally:
    panel.close();until(lambda:panel.process is None,35)
    node.destroy_node();rclpy.try_shutdown()
