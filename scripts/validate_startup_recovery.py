"""所有する起動プロセスへ異常終了を注入して復旧と再走行禁止を確認。"""
import os
os.environ['QT_QPA_PLATFORM']='offscreen';os.environ.setdefault('ROS_DOMAIN_ID','42')
import time,signal,json
import rclpy
from PyQt5 import QtWidgets
from course_selector import CoursePanel,PanelNode,ROOT,configure_japanese_font
app=QtWidgets.QApplication([]);configure_japanese_font(app);rclpy.init();node=PanelNode();panel=CoursePanel(node,simulation_gui=False);panel.show()
def until(test,seconds=150):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  app.processEvents()
  if test():return
  time.sleep(.02)
 raise TimeoutError(panel.course_status.text())
try:
 panel.switch_course();first=panel.process.pid
 # 起動直後に所有するプロセス群を終了させる。
 time.sleep(.8);os.killpg(first,signal.SIGINT)
 until(lambda:panel.process is not None and panel.process.pid!=first and not panel.starting)
 assert panel.startup_retries==1 and node.mode=='manual' and abs(node.actual_speed)<.02
 second=panel.process.pid;print('PASS startup failure -> one retry -> manual stop',flush=True)
 # 起動完了後の異常終了は勝手に再起動・再走行させない。
 os.killpg(second,signal.SIGINT);until(lambda:panel.process is None,40)
 started=time.monotonic();until(lambda:time.monotonic()-started>3,5);assert panel.process is None
 (ROOT/'logs/startup_recovery_validation.json').write_text(json.dumps(dict(passed=True,startup_retries=1,runtime_restart=False)))
 print('PASS runtime exit -> remains stopped',flush=True)
finally:
 panel.close();until(lambda:panel.process is None,35);node.destroy_node();rclpy.try_shutdown()
