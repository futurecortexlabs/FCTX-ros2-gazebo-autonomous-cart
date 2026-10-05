"""各コースと表示ありAMCLを起動・正常終了し、Gazeboの異常終了がないことを確認。"""
import os
os.environ['QT_QPA_PLATFORM']='offscreen';os.environ.setdefault('ROS_DOMAIN_ID','42')
import time,json
import rclpy
from PySide2 import QtWidgets
from course_selector import CoursePanel,PanelNode,COURSES,ROOT,configure_japanese_font
app=QtWidgets.QApplication([]);configure_japanese_font(app);rclpy.init();node=PanelNode();panel=CoursePanel(node,simulation_gui=False);panel.show();results=[]
def until(test,seconds=90):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  app.processEvents()
  if test():return
  time.sleep(.02)
 raise TimeoutError(panel.course_status.text())
try:
 cases=[(i,False,'simulation') for i in range(len(COURSES))]+[(next(i for i,c in enumerate(COURSES) if c[1]=='warehouse_course.sdf'),True,'localization')]
 for index,gui,source in cases:
  logfile=ROOT/'logs/course_selector.log';offset=logfile.stat().st_size if logfile.exists() else 0
  panel.simulation_gui=gui;panel.pose_source.setCurrentIndex(panel.pose_source.findData(source));panel.combo.setCurrentIndex(index);panel.switch_course()
  until(lambda:panel.process is not None and not panel.starting and panel.stopping_since is None)
  assert node.mode=='manual' and abs(node.actual_speed)<.02
  started=time.monotonic();until(lambda:time.monotonic()-started>3,5)
  panel.stop_course();until(lambda:panel.process is None,40)
  with logfile.open('rb') as f:f.seek(offset);output=f.read().decode(errors='replace')
  assert not any(word in output for word in ['process has died','failed to terminate','Segmentation fault','Cannot shutdown']),output[-6000:]
  assert 'data: true' in output and 'process has finished cleanly' in output,output[-4000:]
  results.append(dict(course=COURSES[index][1],gui=gui,source=source,passed=True))
  (ROOT/'logs/gazebo_shutdown_validation.json').write_text(json.dumps(results,indent=2));print('PASS clean shutdown',results[-1],flush=True)
finally:
 panel.close();until(lambda:panel.process is None,35);node.destroy_node();rclpy.try_shutdown()
