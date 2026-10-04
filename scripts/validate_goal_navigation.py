"""実際のGazeboで目的地到着と到着後停止を測る。通常のシミュレーションは先に終了する。"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
os.environ.setdefault('ROS_DOMAIN_ID','42')
import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import SetBool,Trigger

ROOT=Path(__file__).resolve().parents[1]


def run_case(world,goal,auto_speed=.45):
    stem='goal_'+world+(f'_{auto_speed:.2f}' if auto_speed != .45 else '')
    log=(ROOT/'logs'/(stem+'.log')).open('w')
    report=ROOT/'logs'/(stem+'.json')
    proc=subprocess.Popen(['bash',str(ROOT/'run_loop.sh'),'gui:=false','controls:=false',
        'mode:=manual',f'auto_speed:={auto_speed}',f'world:={ROOT/"worlds"/world}',f'report:={report}'],
        cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=rclpy.create_node('goal_validation')
    status={};pub=node.create_publisher(PoseStamped,'/loop/goal',1)
    def receive(msg):status.update(json.loads(msg.data))
    node.create_subscription(String,'/loop/navigation_status',receive,1)
    mode=node.create_client(SetBool,'/loop/set_manual')
    def spin_until(test,seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if proc.poll() is not None:raise RuntimeError('Launch exited')
            if test():return
        raise TimeoutError(dict(status))
    try:
        spin_until(lambda:status.get('pose') and status.get('world','').endswith(world),30)
        msg=PoseStamped();msg.header.frame_id='world';msg.pose.position.x=float(goal[0]);msg.pose.position.y=float(goal[1]);msg.pose.orientation.w=1.
        pub.publish(msg)
        spin_until(lambda:status.get('goal')==list(goal) and status.get('state') in ('navigating','paused','rejected'),12)
        assert status['state']!='rejected',status
        assert mode.wait_for_service(timeout_sec=5)
        fut=mode.call_async(SetBool.Request(data=False));rclpy.spin_until_future_complete(node,fut,timeout_sec=5);assert fut.result().success
        print('START',world,goal,'path points',len(status['path']),flush=True)
        deadline=time.monotonic()+240;last=0
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            if status.get('state') in ('blocked','rejected'):raise AssertionError(status)
            if report.exists():
                data=json.loads(report.read_text());assert data['collision_count']==0,data
            if time.monotonic()-last>10:
                print(world,status.get('state'),'distance',status.get('distance'),flush=True);last=time.monotonic()
            if status.get('state')=='arrived':break
        else:raise TimeoutError(status)
        arrival=tuple(status['pose']);ended=time.monotonic()
        spin_until(lambda:time.monotonic()-ended>3.,5)
        data=json.loads(report.read_text())
        error=math.dist(status['pose'][:2],goal)
        assert error<=.20,(error,status)
        assert math.dist(arrival[:2],status['pose'][:2])<.04,status
        assert abs(data['actual_velocity'][0])<.02,data
        assert data['collision_count']==0,data
        print('PASS',world,'error',round(error,4),'contacts',data['collision_count'],flush=True)
        return dict(world=world,goal=goal,error_m=error,**data)
    finally:
        node.destroy_node();rclpy.try_shutdown()
        if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
        try:proc.wait(timeout=24)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=5)
        log.close()


if __name__=='__main__':
    results=[]
    for world,goal in [('obstacle_course.sdf',(-4.3,1.7)),('warehouse_course.sdf',(5.,3.)),
                       ('oval_course.sdf',(-6.,0.)),('loop_course.sdf',(-4.5,0.))]:
        results.append(run_case(world,goal))
        (ROOT/'logs/goal_navigation_validation.json').write_text(json.dumps({'cases':results,'all_passed':len(results)==4},indent=2)+'\n')
