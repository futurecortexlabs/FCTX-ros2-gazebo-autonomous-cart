"""配送順序、待機時間、一時停止、失敗時の停止保持を検証する。"""
import unittest
from pathlib import Path
from goal_navigation import GoalNavigator
from world_clearance import WorldClearance
from mission_control import Mission,validate_route
ROOT=Path(__file__).resolve().parents[1]

class MissionTests(unittest.TestCase):
    def setUp(self):
        self.nav=GoalNavigator(WorldClearance(ROOT/'worlds/warehouse_course.sdf'));self.m=Mission(self.nav)
        self.pose=(0.,-3.4,0.);self.points=[dict(x=0.,y=-3.4,wait=.5),dict(x=1.,y=-3.4,wait=0.)]
    def test_sequence_wait_and_completion_hold(self):
        self.assertTrue(self.m.start(self.points,self.pose,0))
        self.nav.state='arrived';self.m.tick(self.pose,.1,True)
        self.assertEqual(self.m.state,'waiting')
        self.m.tick(self.pose,.35,True);self.assertEqual(self.m.index,0)
        self.m.tick(self.pose,.6,True);self.assertEqual(self.m.index,1)
        self.nav.state='arrived';self.m.tick((1.,-3.4,0.),.7,True)
        self.assertEqual(self.m.state,'completed');self.assertTrue(self.nav.hold)
        self.assertEqual(self.nav.command((1.,-3.4,0.),.45,1),(0.,0.))
    def test_pause_and_stop_freeze_wait(self):
        self.m.start(self.points,self.pose,0);self.nav.state='arrived';self.m.tick(self.pose,.1,True)
        self.m.paused=True;self.m.tick(self.pose,100,True);self.assertEqual(self.m.remaining,.5)
        self.m.paused=False;self.m.tick(self.pose,101,False);self.assertEqual(self.m.remaining,.5)
        self.m.tick(self.pose,101.1,True);self.assertAlmostEqual(self.m.remaining,.4)
    def test_preflight_rejects_later_obstacle(self):
        self.assertFalse(self.m.start(self.points+[dict(x=2.5,y=1.4,wait=0)],self.pose,0))
        self.assertEqual(self.m.state,'failed');self.assertEqual(self.nav.path,[])
    def test_cancel_and_block(self):
        self.m.start(self.points,self.pose,0);self.m.cancel()
        self.assertFalse(self.m.active);self.assertTrue(self.nav.hold)
        self.m.start(self.points,self.pose,0);self.nav.state='blocked';self.m.tick(self.pose,1,True)
        self.assertEqual(self.m.state,'failed')
    def test_invalid_routes(self):
        for points in [[],[dict(x=float('nan'),y=0,wait=0)],[dict(x=0,y=0,wait=-1)],self.points*26]:
            with self.assertRaises(ValueError):validate_route(points)

if __name__=='__main__':unittest.main()
