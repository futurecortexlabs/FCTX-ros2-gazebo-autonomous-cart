"""目的地計画・失敗時停止・到着後の保持を、ROSなしで検証する。"""
from pathlib import Path
import math
import time
import unittest
from goal_navigation import GoalNavigator
from world_clearance import WorldClearance

ROOT=Path(__file__).resolve().parents[1]


class GoalTests(unittest.TestCase):
    def nav(self,world='warehouse_course.sdf'):
        return GoalNavigator(WorldClearance(ROOT/'worlds'/world))

    def test_detour_and_safe_segments(self):
        nav=self.nav();start=(0.,-3.4,0.);goal=(5.,3.)
        self.assertTrue(nav.set_goal(start,goal,0),nav.detail)
        self.assertGreater(len(nav.path),2)
        for a,b in zip(nav.path,nav.path[1:]):self.assertTrue(nav.map.visible(a,b,.20))
        self.assertEqual(nav.path[-1],goal)

    def test_invalid_and_unreachable_goals_stop(self):
        nav=self.nav('loop_course.sdf')
        for goal in [(3.,0.),(30.,0.),(0.,0.)]:
            self.assertFalse(nav.set_goal((4.5,0.,0.),goal,0),goal)
            self.assertEqual(nav.command((4.5,0.,0.),.45,1),(0.,0.))
            self.assertTrue(nav.hold)

    def test_arrival_and_cancel_hold(self):
        nav=self.nav();self.assertTrue(nav.set_goal((0.,-3.4,0.),(0.,-3.3),0))
        self.assertEqual(nav.command((0.,-3.4,0.),.45,1),(0.,0.))
        self.assertEqual(nav.state,'arrived')
        self.assertEqual(nav.command((0.,-3.4,0.),.45,2),(0.,0.))
        nav.cancel();self.assertTrue(nav.hold);self.assertEqual(nav.state,'cancelled')
        nav.cancel(free=True);self.assertFalse(nav.hold)

    def test_pause_and_block(self):
        nav=self.nav();nav.set_goal((0.,-3.4,0.),(1.,-3.4),0)
        self.assertEqual(nav.command((0.,-3.4,0.),0.,2),(0.,0.))
        self.assertEqual(nav.state,'paused')
        self.assertGreater(nav.command((0.,-3.4,0.),.45,3)[0],0)
        self.assertEqual(nav.command((0.,-3.4,0.),.45,16),(0.,0.))
        self.assertEqual(nav.state,'blocked')

    def test_seven_maps_plan_without_crossing_walls(self):
        for file,start,goal in [('loop_course.sdf',(4.5,0,math.pi/2),(-4.5,0)),
                                ('obstacle_course.sdf',(4.5,0,math.pi/2),(-4.3,1.7)),
                                ('oval_course.sdf',(6,0,math.pi/2),(-6,0)),
                                ('warehouse_course.sdf',(0,-3.4,0),(5,3)),
                                ('large_oval_course.sdf',(10,0,math.pi/2),(-10,0)),
                                ('large_warehouse_course.sdf',(-10,-7.5,0),(10,7.5)),
                                ('slalom_course.sdf',(-10,-5,math.pi/2),(10,5))]:
            nav=self.nav(file);began=time.monotonic()
            self.assertTrue(nav.set_goal(start,goal,0),nav.detail)
            for a,b in zip(nav.path,nav.path[1:]):self.assertTrue(nav.map.visible(a,b,.20))
            print(file,'points',len(nav.path),'seconds',round(time.monotonic()-began,3))


if __name__=='__main__':unittest.main()
