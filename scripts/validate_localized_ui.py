"""AMCLの実画面操作・推定断停止・配送完了をGazeboで検証する。"""
import os
os.environ['QT_QPA_PLATFORM']='offscreen';os.environ.setdefault('ROS_DOMAIN_ID','42')
import time,json,signal
from pathlib import Path
import rclpy
from PySide2 import QtWidgets
from course_selector import CoursePanel,PanelNode,COURSES,ROOT,configure_japanese_font
app=QtWidgets.QApplication([]);configure_japanese_font(app);rclpy.init();node=PanelNode();panel=CoursePanel(node,simulation_gui=False);panel.show();paused_pid=None

def until(test,seconds=45):
 end=time.monotonic()+seconds;last=0
 while time.monotonic()<end:
  app.processEvents()
  if test():return
  if time.monotonic()-last>10:print(node.state,panel.navigation.data.get('state'),panel.navigation.data.get('detail'),flush=True);last=time.monotonic()
  time.sleep(.02)
 raise TimeoutError(panel.navigation.data)
try:
 panel.pose_source.setCurrentIndex(panel.pose_source.findData('localization'))
 panel.combo.setCurrentIndex(next(i for i,c in enumerate(COURSES) if c[1]=='warehouse_course.sdf'));panel.switch_course()
 until(lambda:panel.navigation.data.get('frame')=='map' and bool(panel.navigation.data.get('pose')) and panel.navigation.map.map_image is not None)
 # 制御と安全ゲートは正解位置を購読していないことをROSグラフでも確認。
 names=[(info.node_name,info.node_namespace) for info in node.get_subscriptions_info_by_topic('/loop/ground_truth')]
 assert all(n not in ('loop_autonomous_driver','loop_safety_gate','loop_localization_bridge','ekf_filter_node') for n,_ in names),names
 panel.navigation.select(0.,2.);panel.navigation.go();until(lambda:abs(node.actual_speed)>.1)
 for p in Path('/proc').iterdir():
  if not p.name.isdigit():continue
  try:
   if b'/nav2_amcl/amcl' in (p/'cmdline').read_bytes() and os.getpgid(int(p.name))==panel.process.pid:paused_pid=int(p.name);break
  except (OSError,ProcessLookupError):pass
 assert paused_pid is not None
 os.kill(paused_pid,signal.SIGSTOP)
 until(lambda:node.state=='waiting_for_fresh_data' and abs(node.actual_speed)<.02,8)
 print('PASS AMCL suspended -> safety stop without ground truth fallback',flush=True)
 os.kill(paused_pid,signal.SIGCONT);paused_pid=None
 until(lambda:panel.navigation.data.get('state')=='arrived',150)
 ui=panel.navigation.mission
 for x,y in [(0.,0.),(0.,2.)]:panel.navigation.select(x,y);ui.add()
 saved=ROOT/'routes/warehouse_slam_delivery.json';ui.write_route(saved);ui.table.setRowCount(0);ui.read_route(saved)
 assert json.loads(saved.read_text())['frame']=='map'
 ui.send('start');until(lambda:panel.navigation.data.get('mission',{}).get('state')=='completed',220)
 t=time.monotonic();until(lambda:time.monotonic()-t>3,5)
 report=json.loads((ROOT/'logs/loop_status.json').read_text());assert report['collision_count']==0,report
 assert abs(node.actual_speed)<.02
 panel.navigation.findChild(QtWidgets.QTabWidget).setCurrentIndex(1);app.processEvents();panel.grab().save(str(ROOT/'logs/localized_mission_ui.png'))
 (ROOT/'logs/localized_ui_validation.json').write_text(json.dumps(dict(passed=True,ground_truth_subscribers=names,report=report),indent=2))
 print('PASS map display, saved map-frame route, AMCL delivery and final stop',flush=True)
finally:
 if paused_pid is not None:os.kill(paused_pid,signal.SIGCONT)
 panel.close();until(lambda:panel.process is None,35);node.destroy_node();rclpy.try_shutdown()
