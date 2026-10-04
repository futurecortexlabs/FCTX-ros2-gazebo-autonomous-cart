"""SLAM/AMCLで推定位置による走行と正解位置との差を外部から検証する。"""
import os
os.environ.setdefault('ROS_DOMAIN_ID','42')
import json,time,math,sys
import rclpy
from geometry_msgs.msg import PoseStamped,Twist
from nav_msgs.msg import Odometry,OccupancyGrid
from std_msgs.msg import String
from std_srvs.srv import SetBool
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
rclpy.init();node=rclpy.create_node('slam_external_validation');status={};truth=[];maps=[]
node.create_subscription(String,'/loop/navigation_status',lambda m:status.update(json.loads(m.data)),10)
node.create_subscription(Odometry,'/loop/ground_truth',lambda m:truth.append(m),qos_profile_sensor_data)
node.create_subscription(OccupancyGrid,'/map',lambda m:maps.append(m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
pub=node.create_publisher(PoseStamped,'/loop/goal',10);mode=node.create_client(SetBool,'/loop/set_manual')
def until(test,seconds):
 end=time.monotonic()+seconds;last=0
 while time.monotonic()<end:
  rclpy.spin_once(node,timeout_sec=.05)
  if test():return
  if time.monotonic()-last>10:print(status.get('state'),status.get('detail'),status.get('pose'),flush=True);last=time.monotonic()
 raise TimeoutError(status)
try:
 until(lambda:bool(status.get('pose')) and bool(maps),30)
 assert status['frame']=='map';print('MAP',maps[-1].info.width,maps[-1].info.height,'initial pose',status['pose'],flush=True)
 if status['pose_source']=='slam':
  assert mode.wait_for_service(timeout_sec=5);f=mode.call_async(SetBool.Request(data=True));rclpy.spin_until_future_complete(node,f,timeout_sec=5)
  manual=node.create_publisher(Twist,'/loop/cmd_manual',10)
  total=0.;previous=status['pose'][2];end=time.monotonic()+90
  while total<6.5 and time.monotonic()<end:
   cmd=Twist();cmd.angular.z=.4;manual.publish(cmd)
   rclpy.spin_once(node,timeout_sec=.05)
   yaw=status['pose'][2];total+=abs(math.atan2(math.sin(yaw-previous),math.cos(yaw-previous)));previous=yaw
   time.sleep(.03)
  manual.publish(Twist());assert total>=6.5,total
  t=time.monotonic();until(lambda:time.monotonic()-t>3,5)
  print('PASS SLAM scan rotation',flush=True)
 results=[]
 for target in [(0.,2.),(0.,0.)]:
  msg=PoseStamped();msg.header.frame_id='map';msg.pose.position.x=target[0];msg.pose.position.y=target[1];msg.pose.orientation.w=1.;pub.publish(msg)
  issued=time.monotonic();until(lambda:status.get('goal')==list(target) and (status.get('state')!='rejected' or time.monotonic()-issued>3),15);assert status['state']!='rejected',status
  assert mode.wait_for_service(timeout_sec=5);f=mode.call_async(SetBool.Request(data=False));rclpy.spin_until_future_complete(node,f,timeout_sec=5);assert f.result().success
  until(lambda:status.get('state') in ('arrived','blocked','rejected'),160)
  assert status['state']=='arrived',status
  t=time.monotonic();until(lambda:time.monotonic()-t>3,5)
  actual=truth[-1].pose.pose.position;estimate=status['pose']
  error=math.hypot(estimate[0]-actual.x,estimate[1]-(actual.y+3.4))
  assert error<.35,(estimate,actual,error)
  report=json.loads((ROOT/'logs/loop_status.json').read_text());assert report['collision_count']==0
  results.append(dict(target=target,estimated=estimate,truth_xy=[actual.x,actual.y],error_m=error));print('PASS',results[-1],flush=True)
 (ROOT/('logs/'+status['pose_source']+'_stage_validation.json')).write_text(json.dumps(dict(passed=True,cases=results,map_cells=len(maps[-1].data)),indent=2))
finally:node.destroy_node();rclpy.try_shutdown()
