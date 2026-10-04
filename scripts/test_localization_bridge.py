"""自己位置の品質・TF更新停止を疑似入力で検証する。"""
import os
os.environ['ROS_DOMAIN_ID']='50'
import unittest,time
from unittest.mock import Mock
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from localization_bridge import LocalizationBridge
class BridgeTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):rclpy.init()
 @classmethod
 def tearDownClass(cls):rclpy.shutdown()
 def setUp(self):
  self.node=LocalizationBridge('localization');n=self.node
  n.wheel=Odometry();n.wheel.header.stamp=n.get_clock().now().to_msg();n.last_wheel=time.monotonic()
  n.covariance=[0.]*36;n.covariance[0]=n.covariance[7]=.01;n.covariance[35]=.01
  n.last_imu=time.monotonic();n.last_correction=time.monotonic();self.tf=TransformStamped();self.tf.header.stamp=n.wheel.header.stamp;self.tf.transform.rotation.w=1.
  n.buffer.lookup_transform=Mock(return_value=self.tf);n.pub=Mock();n.status=Mock()
 def tearDown(self):self.node.destroy_node()
 def test_valid_publishes(self):
  self.node.tick();self.node.pub.publish.assert_called_once()
 def test_uncertain_or_stale_does_not_publish(self):
  self.node.covariance[0]=1.;self.node.tick();self.node.pub.publish.assert_not_called()
  self.node.covariance[0]=.01;self.tf.header.stamp.sec-=3;self.node.tick();self.node.pub.publish.assert_not_called()
 def test_moving_without_scan_correction_stops(self):
  self.node.wheel.twist.twist.linear.x=.2;self.node.last_correction=0.;self.node.moving_since=time.monotonic()-2.;self.node.tick();self.node.pub.publish.assert_not_called()
 def test_correction_fault_stays_stopped_until_new_correction(self):
  from geometry_msgs.msg import PoseWithCovarianceStamped
  self.node.correction_stale=True;self.node.tick();self.node.pub.publish.assert_not_called()
  self.node.correction(PoseWithCovarianceStamped());self.node.tick();self.node.pub.publish.assert_called_once()
 def test_restart_after_stationary_gets_correction_grace(self):
  self.node.last_correction=0.;self.node.wheel.twist.twist.linear.x=.2;self.node.tick();self.node.pub.publish.assert_called_once()
 def test_stale_imu_stops_even_when_filter_predicts(self):
  self.node.last_imu=0.;self.node.tick();self.node.pub.publish.assert_not_called()
 def test_invalid_imu_does_not_refresh(self):
  from sensor_msgs.msg import Imu
  msg=Imu();msg.orientation.w=0.;msg.header.stamp.sec=1;self.node.last_imu=0.
  self.node.imu(msg);self.assertEqual(self.node.last_imu,0.)
  msg.orientation.w=1.;self.node.imu(msg);self.assertGreater(self.node.last_imu,0.)
  self.node.last_imu=0.;self.node.imu(msg);self.assertEqual(self.node.last_imu,0.)
 def test_nonfinite_rejected(self):
  self.node.covariance[0]=float('nan');self.node.tick();self.node.pub.publish.assert_not_called()
if __name__=='__main__':unittest.main()
