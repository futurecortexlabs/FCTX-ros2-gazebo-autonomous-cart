"""map→baseのTFと車輪速度から制御用Odometryを発行。正解位置は購読しない。"""
import math,time,argparse
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener,TransformException

class LocalizationBridge(Node):
    def __init__(self,source):
        super().__init__('loop_localization_bridge');self.source=source;self.wheel=None;self.last_wheel=0.;self.covariance=None;self.last_correction=0.
        self.last_imu=0.;self.imu_stamp=None;self.moving_since=None;self.correction_stale=False
        self.create_subscription(Imu,'/loop/imu',self.imu,qos_profile_sensor_data)
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.pub=self.create_publisher(Odometry,'/loop/localized_odom',10)
        self.status=self.create_publisher(String,'/loop/localization_status',10)
        self.create_subscription(Odometry,'/loop/wheel_odom',self.odom,qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose' if source=='localization' else '/pose',self.correction,10)
        self.create_timer(.05,self.tick)
    def imu(self,msg):
        stamp=(msg.header.stamp.sec,msg.header.stamp.nanosec)
        values=(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w,msg.angular_velocity.z)
        if stamp!=self.imu_stamp and all(math.isfinite(v) for v in values) and .9 < sum(v*v for v in values[:4]) < 1.1:
            self.imu_stamp=stamp;self.last_imu=time.monotonic()
    def correction(self,msg):
        self.covariance=msg.pose.covariance;self.last_correction=time.monotonic();self.correction_stale=False
    def odom(self,msg):self.wheel=msg;self.last_wheel=time.monotonic()
    def tick(self):
        try:
            if self.wheel is None or time.monotonic()-self.last_wheel>.3:raise ValueError('車輪位置待ち')
            if time.monotonic()-self.last_imu>.3:raise ValueError('IMU更新待ち。停止して再確認します')
            correction=self.buffer.lookup_transform('map','wheel_odom',Time())
            age=(self.get_clock().now().nanoseconds-Time.from_msg(correction.header.stamp).nanoseconds)/1e9
            if age>1.:raise ValueError('位置補正が更新されていません')
            moving=abs(self.wheel.twist.twist.linear.x)>.02 or abs(self.wheel.twist.twist.angular.z)>.04
            # 停止中は照合が間引かれる。動き始めには次の照合まで猶予を与え、走行中の断は監視する。
            if not moving:self.moving_since=None
            elif self.moving_since is None:self.moving_since=time.monotonic()
            if moving and time.monotonic()-max(self.last_correction,self.moving_since)>1.5:self.correction_stale=True
            if self.correction_stale:raise ValueError('位置照合が更新されていません。新しい照合まで停止します')
            if self.source=='localization':
                if self.covariance is None or not all(math.isfinite(self.covariance[i]) and self.covariance[i]>=0 for i in (0,7,35)) or max(self.covariance[0],self.covariance[7])>.25 or self.covariance[35]>.3:
                    raise ValueError('自己位置が未確定です。初期位置を指定してください')
            tf=self.buffer.lookup_transform('map','loop_cart/base_link',Time())
            values=(tf.transform.translation.x,tf.transform.translation.y,tf.transform.rotation.z,tf.transform.rotation.w)
            if not all(math.isfinite(v) for v in values):raise ValueError('推定位置が不正です')
            if (Time.from_msg(self.wheel.header.stamp).nanoseconds-Time.from_msg(tf.header.stamp).nanoseconds)/1e9>.3:raise ValueError('位置TFが古いため停止します')
            msg=Odometry();msg.header.stamp=self.wheel.header.stamp;msg.header.frame_id='map';msg.child_frame_id='loop_cart/base_link'
            msg.pose.pose.position.x=tf.transform.translation.x;msg.pose.pose.position.y=tf.transform.translation.y
            msg.pose.pose.orientation=tf.transform.rotation;msg.twist=self.wheel.twist
            if self.covariance is not None:msg.pose.covariance=self.covariance
            self.pub.publish(msg);self.status.publish(String(data='SLAM位置推定中' if self.source=='slam' else 'AMCL自己位置推定中'))
        except (ValueError,TransformException) as exc:self.status.publish(String(data=str(exc)))

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--source',required=True);args,ros=parser.parse_known_args()
    import signal
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=ros,signal_handler_options=SignalHandlerOptions.NO);node=LocalizationBridge(args.source)
    stopping=[False]
    def request_stop(*_):stopping[0]=True
    signal.signal(signal.SIGINT,request_stop);signal.signal(signal.SIGTERM,request_stop)
    try:
        while rclpy.ok() and not stopping[0]:rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
