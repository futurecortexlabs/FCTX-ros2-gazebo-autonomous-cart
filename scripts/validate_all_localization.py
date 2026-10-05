"""7コースのSLAM観測・保存・AMCL再起動・往復配送を実際の操作経路で検証。"""
import os
os.environ['QT_QPA_PLATFORM']='offscreen';os.environ.setdefault('ROS_DOMAIN_ID','42')
import time,json,math,sys
from pathlib import Path
import rclpy
from PySide2 import QtWidgets
from course_selector import CoursePanel,PanelNode,COURSES,ROOT,configure_japanese_font
from save_slam_map import save
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
import xml.etree.ElementTree as ET
app=QtWidgets.QApplication([]);configure_japanese_font(app);rclpy.init();node=PanelNode();panel=CoursePanel(node,simulation_gui=False);panel.show()
results=[];truth=None
def receive_truth(msg):
 global truth
 truth=msg
node.create_subscription(Odometry,'/loop/ground_truth',receive_truth,qos_profile_sensor_data)
def position_error():
 assert truth is not None
 spawn=list(map(float,ET.parse(ROOT/'worlds'/COURSES[panel.active_course][1]).find(".//model[@name='loop_cart']/pose").text.split()))
 sx,sy,_,_,_,yaw=spawn;p=truth.pose.pose.position;estimate=panel.navigation.data['pose']
 x=(p.x-sx)*math.cos(yaw)+(p.y-sy)*math.sin(yaw);y=-(p.x-sx)*math.sin(yaw)+(p.y-sy)*math.cos(yaw)
 return math.hypot(estimate[0]-x,estimate[1]-y)

def report():
 try:return json.loads((ROOT/'logs/loop_status.json').read_text())
 except (OSError,ValueError):return {}
def until(test,seconds=60):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  app.processEvents()
  if report().get('collision_count',0):raise AssertionError('contact detected')
  if test():return
  time.sleep(.02)
 raise TimeoutError(dict(status=panel.course_status.text(),navigation=panel.navigation.data,localization=panel.localization_status.text()))
def pause(seconds):
 start=time.monotonic();until(lambda:time.monotonic()-start>seconds,seconds+5)
def start(index,source):
 panel.pose_source.setCurrentIndex(panel.pose_source.findData(source));panel.combo.setCurrentIndex(index);panel.switch_course()
 until(lambda:panel.active_course==index and panel.active_pose_source==source and not panel.starting and panel.stopping_since is None and panel.process is not None,150)
 assert node.mode=='manual' and abs(node.actual_speed)<.02
 print('READY',COURSES[index][1],source,flush=True)
def mission():
 ui=panel.navigation.mission;ui.table.setRowCount(0)
 for x,y in [(1.2,0.),(0.,0.)]:panel.navigation.select(x,y);ui.add()
 ui.send('start');until(lambda:panel.navigation.data.get('mission',{}).get('state') in ('running','failed'),20)
 assert panel.navigation.data['mission']['state']=='running',panel.navigation.data
 until(lambda:panel.navigation.data.get('mission',{}).get('state') in ('completed','failed'),160)
 assert panel.navigation.data['mission']['state']=='completed',panel.navigation.data
 pause(2);assert abs(node.actual_speed)<.02
 data=report();data['endpoint_error_m']=position_error();assert data['endpoint_error_m']<.35,data
 return data
try:
 selected=sys.argv[1:]
 for index,(_,filename,_) in enumerate(COURSES):
  if selected and filename not in selected:continue
  start(index,'slam')
  # 押下操作を保持して一周分観測する。制御に正解位置は渡さない。
  previous=panel.navigation.data['pose'][2];total=0.;began=time.monotonic()
  panel.turn_speed.set_value(.35);panel.held_buttons.add('left')
  while total<6.5 and time.monotonic()-began<90:
   app.processEvents();yaw=panel.navigation.data['pose'][2]
   total+=abs(math.atan2(math.sin(yaw-previous),math.cos(yaw-previous)));previous=yaw
   time.sleep(.02)
  panel.clear_input();assert total>=6.5,total;pause(3)
  from occupancy_navigation import OccupancyNavigationMap
  grid=OccupancyNavigationMap(panel.navigation.last_grid)
  print('MAP CLEARANCE',filename,[(xy,grid.geometry.clearance(*xy)) for xy in [(0.,0.),(1.2,0.)]],flush=True)
  path=save(ROOT/'worlds'/filename);pause(.5)
  slam=mission();path=save(ROOT/'worlds'/filename)
  print('SLAM SAVED',filename,path,flush=True)
  start(index,'localization');localized=mission()
  ui=panel.navigation.mission;ui.write_route(ROOT/'routes'/(Path(filename).stem+'_localized_delivery.json'))
  results.append(dict(course=filename,slam=slam,localization=localized,passed=True))
  (ROOT/'logs/all_localization_validation.json').write_text(json.dumps(results,indent=2))
  print('PASS',filename,'SLAM + saved map AMCL delivery, zero contacts',flush=True)
finally:
 panel.close()
 end=time.monotonic()+35
 while panel.process is not None and time.monotonic()<end:app.processEvents();time.sleep(.02)
 node.destroy_node();rclpy.try_shutdown()
