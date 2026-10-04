"""実行中のGazeboへ実験用の箱を追加・削除する。台車近傍と壁内は拒否。"""
import argparse,json,math,os,subprocess,time,uuid
from pathlib import Path
from world_clearance import WorldClearance
ROOT=Path(__file__).resolve().parents[1]

def service(name,kind,request):
    # 指定パーティションを継承し、他のGazebo環境へ追加・削除要求を送らない。
    env=os.environ.copy();env.setdefault('GZ_PARTITION','ros2_gazebo_loop')
    p=subprocess.run(['gz','service','-s','/world/loop_course/'+name,'--reqtype',kind,'--reptype','gz.msgs.Boolean','--timeout','3000','--req',request],env=env,capture_output=True,text=True,timeout=6)
    if p.returncode or 'data: true' not in p.stdout:raise RuntimeError('Gazebo操作に失敗しました: '+p.stderr+p.stdout)

def live_pose():
    # 数秒ごとのログ位置ではなく、追加直前のROS位置・速度を使用する。
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    owned=not rclpy.ok()
    if owned:rclpy.init()
    node=rclpy.create_node('obstacle_placement_check');samples=[]
    node.create_subscription(Odometry,'/loop/ground_truth',samples.append,qos_profile_sensor_data)
    try:
        end=time.monotonic()+2.
        while not samples and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        if not samples:raise ValueError('新しい台車位置を受信できません')
        msg=samples[-1]
        return (msg.pose.pose.position.x,msg.pose.pose.position.y),abs(msg.twist.twist.linear.x)
    finally:
        node.destroy_node()
        if owned:rclpy.shutdown()

def add(x,y):
    report=ROOT/'logs/loop_status.json'
    if time.time()-report.stat().st_mtime>5:raise ValueError('実行中コースを確認してください')
    data=json.loads(report.read_text());pose,speed=live_pose()
    if not pose or not all(math.isfinite(v) for v in (x,y)) or math.dist(pose[:2],(x,y))<2.5+3.*speed:
        raise ValueError('箱を台車から離してください（最低2.5m＋走行速度に応じた余裕）')
    world=Path(data['course_world']);geometry=WorldClearance(world)
    from goal_navigation import NavigationMap
    grid=NavigationMap(geometry)
    if not grid.xmin<x<grid.xmax or not grid.ymin<y<grid.ymax or geometry.clearance(x,y)<.4:
        raise ValueError('箱は壁・棚から離れたコース内に置いてください')
    # 台車と同じ走行可能領域であることを確認し、外壁外や内周島を拒否する。
    grid.plan(pose[:2],(x,y))
    name='obstacle_runtime_'+uuid.uuid4().hex[:10]
    sdf=f'<sdf version="1.9"><model name="{name}"><static>true</static><pose>{x} {y} .6 0 0 0</pose><link name="box"><collision name="obstacle_box"><geometry><box><size>1 1 1.2</size></box></geometry></collision><visual name="visual"><geometry><box><size>1 1 1.2</size></box></geometry><material><ambient>1 .3 .03 1</ambient><diffuse>1 .3 .03 1</diffuse></material></visual></link></model></sdf>'
    service('create','gz.msgs.EntityFactory','sdf: '+json.dumps(sdf))
    registry=ROOT/'logs/runtime_obstacles.json'
    names=json.loads(registry.read_text()) if registry.exists() else []
    names.append(name);registry.write_text(json.dumps(names));return name

def remove_all():
    registry=ROOT/'logs/runtime_obstacles.json'
    names=json.loads(registry.read_text()) if registry.exists() else []
    remaining=[]
    for name in names:
        if not isinstance(name,str) or not name.startswith('obstacle_runtime_'):continue
        try:service('remove','gz.msgs.Entity','name: '+json.dumps(name)+' type: MODEL')
        except RuntimeError:remaining.append(name)
    registry.write_text(json.dumps(remaining))
    # コース再起動で既に消えた名前は失敗を返すこともあるため、残りは次回再確認する。
    return len(names)-len(remaining)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--x',type=float);parser.add_argument('--y',type=float);parser.add_argument('--remove-all',action='store_true');a=parser.parse_args()
    try:
        if a.remove_all:print(f'追加障害物を削除しました: {remove_all()}個')
        elif a.x is not None and a.y is not None:print('障害物を追加しました: '+add(a.x,a.y))
        else:raise ValueError('X・Yを指定してください')
    except Exception as exc:print(str(exc));raise SystemExit(1)
