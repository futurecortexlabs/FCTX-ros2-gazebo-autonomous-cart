"""動的地図の迂回・消去条件・静的壁除外を確認する。"""
import unittest,math
from pathlib import Path
from dynamic_obstacles import DynamicObstacles
from goal_navigation import GoalNavigator
from world_clearance import WorldClearance
from occupancy_navigation import OccupancyNavigationMap
from types import SimpleNamespace as NS
ROOT=Path(__file__).resolve().parents[1]
class DynamicTests(unittest.TestCase):
    def corridor_grid(self):
        # 2 mの中央通路と、両端の広い出発・到着エリアを観測した地図。
        width,height,resolution=201,81,.05
        data=[0]*(width*height)
        for row in (20,60):
            for col in range(80,121):data[row*width+col]=100
        origin=NS(position=NS(x=-5.,y=-2.),orientation=NS(z=0.))
        return NS(info=NS(width=width,height=height,resolution=resolution,origin=origin),data=data)

    def test_new_static_map_does_not_double_inflate_corridor(self):
        grid=OccupancyNavigationMap(self.corridor_grid())
        start,goal=(-3.,0.),(3.,0.)
        self.assertTrue(grid.plan(start,goal))
        tracker=DynamicObstacles(grid.geometry)
        tracker.cells={(0,4):(0.,1.),(0,-4):(0.,-1.)}
        # SLAM取り込み後も同じ壁を動的半径.23 mで重ねると、通れる経路を失う。
        grid.update_obstacles(tracker.points)
        with self.assertRaises(ValueError):grid.plan(start,goal)
        self.assertTrue(tracker.replace_geometry(grid.geometry))
        self.assertFalse(tracker.cells)
        grid.update_obstacles(tracker.points)
        self.assertTrue(grid.plan(start,goal))

    def test_map_reclassification_preserves_unknown_and_unmapped_points(self):
        msg=self.corridor_grid()
        # 右側の未観測セルは、障害物を観測した証拠にしない。
        col=round((2.-(-4.975))/.05)
        for row in range(81):msg.data[row*201+col]=-1
        grid=OccupancyNavigationMap(msg)
        tracker=DynamicObstacles(grid.geometry)
        tracker.cells={(0,4):(0.,1.),(8,0):(2.,0.),(-8,0):(-2.,0.)}
        self.assertTrue(tracker.replace_geometry(grid.geometry))
        self.assertEqual(tracker.cells,{(8,0):(2.,0.),(-8,0):(-2.,0.)})
        self.assertLess(grid.geometry.clearance(2.,0.),0.)
        self.assertFalse(grid.geometry.observed_obstacle_near(2.,0.,.30))
        # 再分類そのものは未観測領域を通行可能にしない。
        self.assertFalse(grid.visible((1.5,0.),(2.5,0.)))

    def test_replan_avoids_added_points(self):
        nav=GoalNavigator(WorldClearance(ROOT/'worlds/large_warehouse_course.sdf'))
        start=(-10.,-7.5,0.);goal=(-3.,-7.5)
        self.assertTrue(nav.set_goal(start,goal,0))
        nav.map.update_obstacles([(-6.5,-7.5),(-6.5,-7.25),(-6.5,-7.75)])
        self.assertFalse(nav.map.visible(start[:2],goal))
        self.assertTrue(nav.set_goal(start,goal,1),nav.detail)
        self.assertGreater(len(nav.path),2)
        for a,b in zip(nav.path,nav.path[1:]):self.assertTrue(nav.map.visible(a,b,.20))
        nav.map.update_obstacles([]);self.assertTrue(nav.map.visible(start[:2],goal))
    def test_occluded_points_retained_and_clear_ray_removes(self):
        tracker=DynamicObstacles(WorldClearance(ROOT/'worlds/warehouse_course.sdf'))
        tracker.cells={(8,0):(2.,0.)}
        tracker.update((0,0,0),[1.]*720,-math.pi,math.tau/720,.06,15.)
        self.assertIn((8,0),tracker.cells)
        tracker.update((0,0,0),[float('inf')]*720,-math.pi,math.tau/720,.06,15.)
        self.assertNotIn((8,0),tracker.cells)
    def test_static_wall_not_added(self):
        tracker=DynamicObstacles(WorldClearance(ROOT/'worlds/warehouse_course.sdf'))
        # 正面の外壁上に当たる、狭い角度範囲の測定。
        angles=[-.01+i*.001 for i in range(21)]
        ranges=[6.92/math.cos(a) for a in angles]
        tracker.update((0,0,0),ranges,-.01,.001,.06,15.)
        self.assertFalse(tracker.points)
if __name__=='__main__':unittest.main()
