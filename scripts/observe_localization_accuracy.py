"""外部検証専用。正解位置は測定だけに使い、制御には配信しない。"""
import os
os.environ.setdefault('ROS_DOMAIN_ID','42')
import rclpy,time,json,math,xml.etree.ElementTree as ET
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
rclpy.init();node=rclpy.create_node('acceptance_accuracy_observer');truth=None;results={};last=0
spawns={p.name:list(map(float,ET.parse(p).find(".//model[@name='loop_cart']/pose").text.split())) for p in (ROOT/'worlds').glob('*course.sdf')}
def actual(msg):
 global truth
 truth=msg
node.create_subscription(Odometry,'/loop/ground_truth',actual,qos_profile_sensor_data)
def estimate(msg):
 global last
 if truth is None:return
 a=truth.header.stamp;b=msg.header.stamp
 if abs((a.sec-b.sec)+(a.nanosec-b.nanosec)*1e-9)>.1:return
 try:report=json.loads((ROOT/'logs/loop_status.json').read_text())
 except (OSError,ValueError):return
 course=Path(report.get('course_world','')).name;source=report.get('localization')
 if course not in spawns or source not in ('slam','localization'):return
 sx,sy,_,_,_,yaw=spawns[course];p=truth.pose.pose.position;q=msg.pose.pose.position
 x=(p.x-sx)*math.cos(yaw)+(p.y-sy)*math.sin(yaw);y=-(p.x-sx)*math.sin(yaw)+(p.y-sy)*math.cos(yaw)
 error=math.hypot(q.x-x,q.y-y);key=course+':'+source
 data=results.setdefault(key,dict(samples=0,max_error_m=0.,sum_error_m=0.,last_error_m=0.))
 data['samples']+=1;data['max_error_m']=max(data['max_error_m'],error);data['sum_error_m']+=error;data['last_error_m']=error
 if time.monotonic()-last>1:
  last=time.monotonic();(ROOT/'logs/acceptance_accuracy.json').write_text(json.dumps(results,indent=2))
node.create_subscription(Odometry,'/loop/localized_odom',estimate,qos_profile_sensor_data)
try:rclpy.spin(node)
finally:node.destroy_node();rclpy.try_shutdown()
