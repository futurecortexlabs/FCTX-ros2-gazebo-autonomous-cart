"""一括距離計算と地図キャッシュが従来の安全判定を保つことを検証する。"""
import os
os.environ['ROS_DOMAIN_ID']='51'
import unittest,math,importlib.util
from pathlib import Path
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from world_clearance import WorldClearance
from dynamic_obstacles import DynamicObstacles
from loop_controller import AutonomousDriver
ROOT=Path(__file__).resolve().parents[1]
class OptimizationTests(unittest.TestCase):
 def test_geometry_and_tracker_equivalence(self):
  reference=ROOT/'tests/reference/dynamic_obstacles_preoptimization.py'
  spec=importlib.util.spec_from_file_location('previous_dynamic',reference)
  old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
  rng=np.random.default_rng(20260919)
  for path in [(ROOT/'worlds'/name) for name in ['loop_course.sdf','obstacle_course.sdf','oval_course.sdf','warehouse_course.sdf','large_oval_course.sdf','large_warehouse_course.sdf','slalom_course.sdf']]:
   geometry=WorldClearance(path)
   xy=rng.uniform(-15,15,(500,2))
   np.testing.assert_allclose(geometry.clearances(xy[:,0],xy[:,1]),[geometry.clearance(x,y) for x,y in xy],atol=1e-12)
   current=DynamicObstacles(geometry);previous=old.DynamicObstacles(geometry)
   for step in range(12):
    ranges=np.repeat(rng.uniform(.1,10,90),8).tolist()
    ranges[0]=math.nan;ranges[2]=math.inf
    if step%4==3:ranges=[math.inf]*720
    args=((step*.1,-3.,step*.05),ranges,-math.pi,math.tau/720,.06,15.)
    self.assertEqual(current.update(*args),previous.update(*args),path.name)
    self.assertEqual(current.cells,previous.cells,path.name)
 def test_map_cache_and_changed_map(self):
  (ROOT/'logs').mkdir(parents=True,exist_ok=True)
  rclpy.init();node=AutonomousDriver(0,ROOT/'logs/cache_test.json',pose_source='slam')
  try:
   msg=OccupancyGrid();msg.header.frame_id='map';msg.info.width=120;msg.info.height=120;msg.info.resolution=.1
   msg.info.origin.orientation.w=1.;msg.data=[0]*14400
   node.receive_map(msg);first=node.navigator.map
   node.dynamic_obstacles.cells[(20,20)]=(2.,2.);first.update_obstacles([(2.,2.)])
   msg.header.stamp.sec=10;node.receive_map(msg)
   self.assertIs(node.navigator.map,first);self.assertEqual([v[:2] for v in first.dynamic],[(2.,2.)])
   msg.data[500]=100;node.receive_map(msg)
   self.assertIsNot(node.navigator.map,first);self.assertEqual([v[:2] for v in node.navigator.map.dynamic],[(2.,2.)])
   changed=node.navigator.map;msg.info.origin.position.x=.1;node.receive_map(msg)
   self.assertIsNot(node.navigator.map,changed)
   geometry=node.geometry;xy=np.random.default_rng(1).uniform(-2,14,(1000,2))
   np.testing.assert_array_equal(geometry.clearances(xy[:,0],xy[:,1]),[geometry.clearance(x,y) for x,y in xy])
  finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':unittest.main()
