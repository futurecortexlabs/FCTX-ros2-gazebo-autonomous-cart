"""実際のQt・ROS・Gazeboで配送、保存読込、一時停止、再開、完了保持を検証。"""
import os
os.environ['QT_QPA_PLATFORM']='offscreen';os.environ.setdefault('ROS_DOMAIN_ID','42')
import json,math,time
import rclpy
from PySide2 import QtWidgets
from course_selector import CoursePanel,PanelNode,COURSES,ROOT,configure_japanese_font
app=QtWidgets.QApplication([]);configure_japanese_font(app);rclpy.init()
node=PanelNode();panel=CoursePanel(node,simulation_gui=False);panel.show()
seen=set();wait_seen=False

def until(test,seconds=45):
    global wait_seen
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        app.processEvents()
        data=panel.navigation.data;mission=data.get('mission',{})
        if mission.get('state')=='failed':raise AssertionError(mission)
        if mission.get('state') in ('running','waiting'):seen.add(mission['index'])
        if mission.get('state')=='waiting':wait_seen=True
        if test():return
        time.sleep(.02)
    raise TimeoutError(panel.navigation.data)

try:
    panel.combo.setCurrentIndex(next(i for i,c in enumerate(COURSES) if c[1]=='large_warehouse_course.sdf'))
    panel.switch_course()
    until(lambda:node.mode=='manual' and panel.navigation.start.isEnabled() and bool(panel.navigation.data.get('pose')))
    ui=panel.navigation.mission
    for x,y in [(-7.,-7.5),(-4.,-7.5),(-10.,-7.5)]:
        panel.navigation.select(x,y);ui.add()
    saved=ROOT/'routes/large_warehouse_delivery.json';saved.parent.mkdir(exist_ok=True)
    ui.write_route(saved);expected=ui.points();ui.table.setRowCount(0);ui.read_route(saved);assert ui.points()==expected
    ui.send('start');until(lambda:node.mode=='auto' and abs(node.actual_speed)>.1)
    ui.send('pause');until(lambda:panel.navigation.data.get('mission',{}).get('paused') and abs(node.actual_speed)<.02)
    pose=panel.navigation.data['pose'];t=time.monotonic();until(lambda:time.monotonic()-t>1.5,3)
    assert math.dist(pose[:2],panel.navigation.data['pose'][:2])<.05
    print('PASS mission UI save/load/start/pause hold',flush=True)
    ui.send('resume');until(lambda:panel.navigation.data.get('mission',{}).get('state')=='completed',200)
    assert seen=={0,1,2} and wait_seen,(seen,wait_seen)
    pose=panel.navigation.data['pose'];t=time.monotonic();until(lambda:time.monotonic()-t>3,5)
    assert math.dist(pose[:2],panel.navigation.data['pose'][:2])<.04
    report=json.loads((ROOT/'logs/loop_status.json').read_text())
    assert report['collision_count']==0,report
    assert abs(node.actual_speed)<.02
    assert math.dist(pose[:2],(-10.,-7.5))<=.20
    panel.navigation.findChild(QtWidgets.QTabWidget).setCurrentIndex(1)
    app.processEvents();panel.grab().save(str(ROOT/'logs/mission_panel.png'))
    (ROOT/'logs/mission_validation.json').write_text(json.dumps(dict(passed=True,visited=sorted(seen),wait_seen=wait_seen,report=report),indent=2))
    print('PASS 3 delivery stops, wait, resume, return, zero contacts, final stop',flush=True)
finally:
    panel.close();until(lambda:panel.process is None,35)
    node.destroy_node();rclpy.try_shutdown()
