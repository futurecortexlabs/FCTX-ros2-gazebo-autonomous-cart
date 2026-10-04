# ファイルの役割: Gazebo・ROSブリッジ・安全ゲート・自動制御・操作画面をまとめて起動する。
from pathlib import Path
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


# 必要な起動引数とプロセス定義を作り、ROS launchへ返す。ここでは制御周期処理は行わない。
def build(context):
    root = Path(__file__).resolve().parents[1]
    script = str(root/'scripts/loop_controller.py')
    source=LaunchConfiguration('pose_source').perform(context)
    map_file=LaunchConfiguration('map_file').perform(context)
    extra=[]
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    env = {'QT_QPA_PLATFORM': 'xcb'}
    # WSLのD3D12ライブラリがスレッド終了前にdlcloseされるとTLS破棄で落ちる。
    # Gazeboプロセスの寿命まで保持し、描画スレッドが終了する前のアンロードを防ぐ。
    d3d12=Path('/usr/lib/wsl/lib/libd3d12core.so')
    if d3d12.is_file():
        # WSL以外のGPU設定は変更せず、WSLでも明示した描画設定を優先する。
        env['GALLIUM_DRIVER'] = os.environ.get('GALLIUM_DRIVER', 'd3d12')
        env['MESA_D3D12_DEFAULT_ADAPTER_NAME'] = os.environ.get('MESA_D3D12_DEFAULT_ADAPTER_NAME', 'NVIDIA')
        env['LD_PRELOAD'] = str(d3d12)+((':'+os.environ['LD_PRELOAD']) if os.environ.get('LD_PRELOAD') else '')
    # 早期中止では描画初期化と破棄が重なるため、Gazeboには破棄完了の猶予を与える。
    server = ExecuteProcess(cmd=['gz', 'sim', '-s', '-r', '-v', '2', world], additional_env=env, output='screen', sigterm_timeout='30', sigkill_timeout='5')
    client = ExecuteProcess(cmd=['gz', 'sim', '-g', '-v', '2'], condition=IfCondition(gui), additional_env=env, output='screen', sigterm_timeout='30', sigkill_timeout='5')
    # SDFのworld名・model名・link名に合わせ、各部位のGazebo接触トピックをROS名へ対応付ける。
    contact_topics = [
        (f'/world/loop_course/model/loop_cart/link/{"base_link" if p == "body" else p}/sensor/{p}_contact/contact', '/loop/contacts/'+p)
        for p in ('body', 'front_left', 'rear_left', 'front_right', 'rear_right')]
    # 「[」はGazeboからROS、「]」はROSからGazeboへの片方向ブリッジを表す。
    bridges = [
        '/loop/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/loop/overview@sensor_msgs/msg/Image[gz.msgs.Image',
        '/loop/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        '/loop/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        '/loop/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        '/loop/wheel_odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
    ] + [gz_topic+'@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts' for gz_topic, _ in contact_topics]
    if source=='simulation':bridges.append('/loop/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V')
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', name='loop_bridge', arguments=bridges, remappings=[('/loop/tf' if source=='simulation' else '/loop/wheel_tf', '/tf')] + contact_topics, output='screen')
    guard = ExecuteProcess(cmd=['/usr/bin/python3', script, '--guard', '--mode', LaunchConfiguration('mode'), '--pose-source', source], output='screen')
    controls = ExecuteProcess(cmd=['/usr/bin/python3', str(root/'scripts/manual_panel.py')], condition=IfCondition(LaunchConfiguration('controls')), additional_env=env, output='screen')
    follower = ExecuteProcess(cmd=['/usr/bin/python3', script, '--pose-source', source, '--world', world, '--auto-speed', LaunchConfiguration('auto_speed'), '--laps', LaunchConfiguration('laps'), '--report', LaunchConfiguration('report')], output='screen')
    static_tf = Node(package='tf2_ros', executable='static_transform_publisher', arguments=[
        '--x', '0', '--y', '0', '--z', '.44', '--frame-id', 'loop_cart/base_link', '--child-frame-id', 'loop_cart/lidar'], parameters=[{'use_sim_time': True}])
    if source!='simulation':
        extra.append(Node(package='robot_localization',executable='ekf_node',name='ekf_filter_node',parameters=[str(root/'config/ekf.yaml')],remappings=[('odometry/filtered','/loop/filtered_odom')],output='screen'))
        estimator=ExecuteProcess(cmd=['/usr/bin/python3',str(root/'scripts/localization_bridge.py'),'--source',source,'--ros-args','-p','use_sim_time:=true'],output='screen')
        extra.append(estimator)
        if source=='slam':
            extra.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('slam_toolbox'))/'launch/online_async_launch.py')),launch_arguments={'slam_params_file':str(root/'config/slam.yaml'),'use_sim_time':'true'}.items()))
        else:
            if not Path(map_file).is_file():raise RuntimeError('保存地図がありません。先にSLAMで地図を保存してください')
            extra.extend([
                Node(package='nav2_map_server',executable='map_server',name='map_server',parameters=[{'use_sim_time':True,'yaml_filename':map_file}],output='screen'),
                Node(package='nav2_amcl',executable='amcl',name='amcl',parameters=[str(root/'config/amcl.yaml')],output='screen'),
                Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='localization_lifecycle',parameters=[{'use_sim_time':True,'autostart':True,'node_names':['map_server','amcl']}],output='screen')])
    actions = [
        DeclareLaunchArgument('world', default_value=str(root/'worlds/loop_course.sdf')),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('controls', default_value=gui, description='Show manual control panel'),
        DeclareLaunchArgument('mode', default_value='auto', choices=['auto', 'manual']),
        DeclareLaunchArgument('auto_speed', default_value='0.45', description='Automatic speed limit, 0.0 to 1.5 m/s'),
        DeclareLaunchArgument('laps', default_value='0', description='0: continuous; positive: optional stop after this many net laps in either direction'),
        DeclareLaunchArgument('report', default_value=str(root/'logs/loop_status.json')),
    ]
    # Any essential process failure stops the whole simulation; closing GUI does too.
    # 主要プロセスが終了したら起動一式を停止する。操作パネル単独の終了は連動対象に含めない。
    for process in (server, bridge, guard, follower, client):
        actions.append(RegisterEventHandler(OnProcessExit(target_action=process, on_exit=lambda event,context: [] if context.is_shutdown else [EmitEvent(event=Shutdown(reason='Loop process exited'))])))
    return actions + [server, client, bridge, static_tf, guard, follower, controls] + extra

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('pose_source',default_value='simulation',choices=['simulation','slam','localization']),DeclareLaunchArgument('map_file',default_value=''),OpaqueFunction(function=build)])
