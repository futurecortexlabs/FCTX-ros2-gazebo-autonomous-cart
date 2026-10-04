"""実際のROSノードで、目的地の受理・拒否・取消・手動切替・古い位置の扱いを検証する。"""
import os
os.environ['ROS_DOMAIN_ID']='48'
import math
import time
import unittest
from pathlib import Path
import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from loop_controller import AutonomousDriver

ROOT=Path(__file__).resolve().parents[1]


class GoalControllerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):rclpy.init()

    @classmethod
    def tearDownClass(cls):rclpy.shutdown()

    def setUp(self):
        self.node=AutonomousDriver(0,ROOT/'logs/goal_controller_test.json',world=str(ROOT/'worlds/warehouse_course.sdf'))
        self.node.pose=(0.,-3.4,0.)
        self.node.pose_time=time.monotonic()

    def tearDown(self):self.node.destroy_node()

    def goal(self,x,y,frame='world'):
        msg=PoseStamped();msg.header.frame_id=frame;msg.pose.position.x=float(x);msg.pose.position.y=float(y)
        self.node.receive_goal(msg)

    def test_accepted_cancel_does_not_resume_roaming(self):
        self.goal(5,3);self.assertEqual(self.node.navigator.state,'navigating')
        self.node.cancel_goal(Trigger.Request(),Trigger.Response())
        self.node.tick();self.assertTrue(self.node.navigator.hold)
        self.assertEqual(self.node.selected_velocity,(0.,0.))
        self.node.free_drive(Trigger.Request(),Trigger.Response())
        self.assertFalse(self.node.navigator.hold)

    def test_reject_bad_frame_coordinates_and_stale_pose(self):
        for x,y,frame in [(2.5,1.4,'world'),(0,0,'robot'),(float('nan'),0,'world')]:
            self.goal(x,y,frame);self.assertEqual(self.node.navigator.state,'rejected')
            self.node.tick();self.assertEqual(self.node.selected_velocity,(0.,0.))
        self.node.pose_time=0.;self.goal(5,3)
        self.assertEqual(self.node.navigator.state,'rejected')

    def test_mission_manual_cancel_and_single_goal_override(self):
        import json
        points=[dict(x=0.,y=-3.,wait=1),dict(x=1.,y=-3.,wait=0)]
        request=dict(id='test',op='start',world=self.node.world,points=points)
        self.node.mission_command(String(data=json.dumps(request)))
        self.assertTrue(self.node.mission.active)
        self.assertTrue(self.node.mission_reply['ok'])
        self.node.control_mode(String(data='manual'))
        self.assertFalse(self.node.mission.active)
        self.node.mission_command(String(data=json.dumps(request)))
        self.goal(5,3)
        self.assertFalse(self.node.mission.active)
        self.assertEqual(self.node.navigator.goal,(5.,3.))

    def test_slam_uses_received_grid_and_map_frame(self):
        from nav_msgs.msg import OccupancyGrid
        self.node.destroy_node()
        self.node=AutonomousDriver(0,ROOT/'logs/slam_unit_test.json',world=str(ROOT/'worlds/warehouse_course.sdf'),pose_source='slam')
        self.assertIsNone(self.node.geometry)
        self.assertIsNone(self.node.navigator.map)
        self.node.pose=(0.,0.,0.);self.node.pose_time=time.monotonic()
        msg=OccupancyGrid();msg.header.frame_id='map';msg.info.width=120;msg.info.height=120;msg.info.resolution=.1
        msg.info.origin.position.x=-6.;msg.info.origin.position.y=-6.;msg.info.origin.orientation.w=1.;msg.data=[0]*14400
        self.node.receive_map(msg)
        self.goal(2,0,'map');self.assertEqual(self.node.navigator.state,'navigating')
        self.goal(2,0,'world');self.assertEqual(self.node.navigator.state,'rejected')

    def test_blocked_dynamic_route_waits_and_recovers(self):
        from sensor_msgs.msg import LaserScan
        self.goal(5,3)
        scan=LaserScan();scan.header.stamp.sec=1
        self.node.scan_data=scan;self.node.pose_stamp=1000000000
        self.node.scan_time=self.node.pose_time=time.monotonic()
        self.node.last_obstacle_update=time.monotonic()
        self.node.navigator.map.update_obstacles([(1.,i*.25) for i in range(-24,25)])
        self.node.update_dynamic_route()
        self.assertEqual(self.node.navigator.state,'waiting_obstacle')
        self.assertEqual(self.node.navigator.command(self.node.pose,.45,time.monotonic()),(0.,0.))
        self.node.navigator.map.update_obstacles([]);self.node.last_replan=0.
        self.node.pose_time=self.node.scan_time=self.node.last_obstacle_update=time.monotonic()
        self.node.update_dynamic_route()
        self.assertEqual(self.node.navigator.state,'navigating')
        self.assertEqual(self.node.replan_count,1)

    def test_slam_map_update_reclassifies_static_hits_and_reopens_route(self):
        from nav_msgs.msg import OccupancyGrid
        from sensor_msgs.msg import LaserScan
        self.node.destroy_node()
        self.node=AutonomousDriver(0,ROOT/'logs/slam_static_hit_test.json',world=str(ROOT/'worlds/warehouse_course.sdf'),pose_source='slam')
        msg=OccupancyGrid();msg.header.frame_id='map'
        msg.info.width=201;msg.info.height=81;msg.info.resolution=.05
        msg.info.origin.position.x=-5.;msg.info.origin.position.y=-2.;msg.info.origin.orientation.w=1.
        msg.data=[0]*(201*81)
        self.node.receive_map(msg)
        self.node.dynamic_obstacles.cells={(0,4):(0.,1.),(0,-4):(0.,-1.)}
        # 次のSLAM地図がLiDARで見つけた2本の壁を正式な占有セルにする。
        data=list(msg.data)
        for row in (20,60):
            for col in range(80,121):data[row*201+col]=100
        msg.data=data
        self.node.receive_map(msg)
        self.assertFalse(self.node.dynamic_obstacles.cells)
        self.assertFalse(self.node.navigator.map.dynamic)
        self.node.pose=(-3.,0.,0.)
        self.node.navigator.hold=True;self.node.navigator.goal=(3.,0.)
        self.node.navigator.state='waiting_obstacle';self.node.navigator.path=[]
        scan=LaserScan();scan.header.stamp.sec=1
        self.node.scan_data=scan;self.node.pose_stamp=1000000000
        self.node.pose_time=self.node.scan_time=self.node.last_obstacle_update=time.monotonic()
        self.node.update_dynamic_route()
        self.assertEqual(self.node.navigator.state,'navigating')
        self.assertEqual(self.node.replan_count,1)
        self.assertTrue(self.node.navigator.map.visible((-3.,0.),(3.,0.),.20))

    def test_mission_rejects_wrong_world(self):
        import json
        self.node.mission_command(String(data=json.dumps(dict(id='bad',op='start',world='other',points=[]))))
        self.assertFalse(self.node.mission_reply['ok'])
        self.assertFalse(self.node.mission.active)

    def test_manual_switch_cancels_active_goal(self):
        self.goal(5,3);self.node.control_mode(String(data='manual'))
        self.assertEqual(self.node.navigator.state,'cancelled')
        self.node.control_mode(String(data='auto'));self.node.tick()
        self.assertEqual(self.node.selected_velocity,(0.,0.))


if __name__=='__main__':unittest.main()
