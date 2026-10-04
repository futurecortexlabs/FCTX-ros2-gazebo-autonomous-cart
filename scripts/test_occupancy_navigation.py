"""占有地図の未知領域・障害物膨張・map座標での計画を検証する。"""
import unittest
from nav_msgs.msg import OccupancyGrid
from occupancy_navigation import OccupancyNavigationMap
class GridTests(unittest.TestCase):
 def grid(self):
  msg=OccupancyGrid();msg.header.frame_id='map';msg.info.width=120;msg.info.height=120;msg.info.resolution=.1
  msg.info.origin.position.x=-6.;msg.info.origin.position.y=-6.;msg.info.origin.orientation.w=1.;msg.data=[0]*14400;return msg
 def test_free_path_and_unknown_barrier(self):
  msg=self.grid();grid=OccupancyNavigationMap(msg);self.assertTrue(grid.plan((-2.,0.),(2.,0.)))
  for row in range(120):msg.data[row*120+60]=-1
  grid=OccupancyNavigationMap(msg)
  with self.assertRaises(ValueError):grid.plan((-2.,0.),(2.,0.))
 def test_obstacle_and_outside_rejected(self):
  msg=self.grid();msg.data[60*120+60]=100;grid=OccupancyNavigationMap(msg)
  for goal in [(0.,0.),(20.,0.)]:
   with self.assertRaises(ValueError):grid.plan((-2.,0.),goal)
 def test_dynamic_overlay_preserves_unknown(self):
  msg=self.grid();msg.data[0]=-1;grid=OccupancyNavigationMap(msg);before=grid.static_clearances.copy()
  grid.update_obstacles([(0.,0.)]);self.assertLess(float(grid.distances(0.,0.)),0)
  grid.update_obstacles([]);self.assertTrue((grid.clearances==before).all())
if __name__=='__main__':unittest.main()
