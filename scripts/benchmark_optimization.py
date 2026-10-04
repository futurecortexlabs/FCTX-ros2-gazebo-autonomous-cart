"""同一入力で最適化前後の処理時間を比較する（Gazebo停止中に実行）。"""
import os
os.environ['ROS_DOMAIN_ID']='51'
import json,math,time,statistics,sys
from pathlib import Path
import numpy as np,yaml
from PIL import Image
import rclpy
from nav_msgs.msg import OccupancyGrid
from world_clearance import WorldClearance
from dynamic_obstacles import DynamicObstacles
from loop_controller import AutonomousDriver
ROOT=Path(__file__).resolve().parents[1]
def measure(fn,n):
 fn();values=[]
 for _ in range(n):
  start=time.perf_counter();fn();values.append((time.perf_counter()-start)*1000)
 return dict(median_ms=statistics.median(values),p95_ms=sorted(values)[int(.95*(n-1))],iterations=n)
rclpy.init();node=AutonomousDriver(0,ROOT/'logs/benchmark_status.json',pose_source='slam')
try:
 geometry=WorldClearance(ROOT/'worlds/large_oval_course.sdf');tracker=DynamicObstacles(geometry)
 ranges=[5.]*720
 data={}
 data['lidar_obstacle_update']=measure(lambda:tracker.update((10.,0.,1.),ranges,-math.pi,math.tau/720,.06,15.),30)
 folder=ROOT/'maps/warehouse_course';meta=yaml.safe_load((folder/'map.yaml').read_text());pixels=np.flipud(np.asarray(Image.open(folder/meta['image'])))
 msg=OccupancyGrid();msg.header.frame_id='map';msg.info.resolution=float(meta['resolution']);msg.info.height,msg.info.width=pixels.shape
 msg.info.origin.position.x=float(meta['origin'][0]);msg.info.origin.position.y=float(meta['origin'][1]);msg.info.origin.orientation.w=1.
 msg.data=np.where(pixels>250,0,np.where(pixels<20,100,-1)).astype(np.int8).ravel().tolist()
 data['identical_map_update']=measure(lambda:node.receive_map(msg),40)
 data['map_size']=[msg.info.width,msg.info.height]
 name=sys.argv[1] if len(sys.argv)>1 else 'latest'
 (ROOT/f'logs/optimization_{name}.json').write_text(json.dumps(data,indent=2))
 print(json.dumps(data,indent=2))
finally:node.destroy_node();rclpy.try_shutdown()
