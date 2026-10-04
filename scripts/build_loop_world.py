#!/usr/bin/env python3
# ファイルの役割: 円形コース・4輪台車・LiDAR・接触センサーをSDFとして生成する。形状を恒久的に変える場合は生成元を編集する。
"""Generate a self-contained circular track and a physical four-wheel cart."""
from pathlib import Path
import math
import xml.etree.ElementTree as E

ROOT = Path(__file__).resolve().parents[1]


# XML要素を作って親へ追加し、必要なら本文も設定して返す。
def tag(parent, element, text=None, **attrs):
    child = E.SubElement(parent, element, attrs)
    if text is not None:
        child.text = str(text)
    return child


# 表示用の色をambientとdiffuseへ設定する。衝突形状は変更しない。
def material(v, color):
    m = tag(v, 'material')
    tag(m, 'ambient', color)
    tag(m, 'diffuse', color)


# 箱または円柱を作る。色の指定がある場合は見た目、ない場合は衝突形状として追加する。
def shape(parent, name, kind, dims, pose='0 0 0 0 0 0', color=None):
    n = tag(parent, 'visual' if color else 'collision', name=name)
    tag(n, 'pose', pose)
    g = tag(tag(n, 'geometry'), kind)
    if kind == 'box':
        tag(g, 'size', dims)
    else:
        tag(g, 'radius', dims[0])
        tag(g, 'length', dims[1])
    if color:
        material(n, color)
    return n


# 剛体の質量と慣性テンソルを設定し、Gazeboの物理計算へ渡す。
def inertia(link, mass, ix, iy, iz):
    i = tag(link, 'inertial')
    tag(i, 'mass', mass)
    t = tag(i, 'inertia')
    for k, v in dict(ixx=ix, iyy=iy, izz=iz, ixy=0, ixz=0, iyz=0).items():
        tag(t, k, v)


# Gazeboシステムのプラグインと設定項目をXMLへ追加する。
def plugin(parent, filename, name, pairs=()):
    p = tag(parent, 'plugin', filename='gz-sim-' + filename + '-system', name='gz::sim::systems::' + name)
    for k, v in pairs:
        tag(p, k, v)
    return p


# 指定した衝突形状に接触センサーを付け、台車の各部位の接触を取得できるようにする。
def contact(link, collision, name):
    s = tag(link, 'sensor', name=name + '_contact', type='contact')
    tag(s, 'always_on', 'true')
    tag(s, 'update_rate', 20)
    tag(s, 'topic', '/loop/contacts/' + name)
    tag(tag(s, 'contact'), 'collision', collision)


# 円形コース・4輪台車・LiDAR・接触センサーをSDFとして生成する。形状を恒久的に変える場合は生成元を編集する。 起動から終了処理までをまとめる入口。
def localization_landmarks(world,a,b):
    """対称な周回壁に非対称の浅い張り出しを設け、LiDARで場所を識別できるようにする。"""
    for i,(angle,length,depth) in enumerate(((-.35,.65,.30),(2.05,1.05,.40),(4.35,1.45,.45))):
        model=tag(world,'model',name=f'obstacle_localization_{i}')
        tag(model,'static','true')
        tangent=math.atan2(b*math.cos(angle),-a*math.sin(angle))
        tag(model,'pose',f'{(a-depth/2)*math.cos(angle)} {(b-depth/2)*math.sin(angle)} .5 0 0 {tangent}')
        link=tag(model,'link',name='landmark')
        shape(link,'marker','box',f'{length} {depth} 1','0 0 0 0 0 0')
        shape(link,'marker_visual','box',f'{length} {depth} 1','0 0 0 0 0 0','.55 .2 .85 1')


def main():
    sdf = E.Element('sdf', version='1.9')
    w = tag(sdf, 'world', name='loop_course')
    ph = tag(w, 'physics', name='physics', type='ignored')
    tag(ph, 'max_step_size', 0.002)
    tag(ph, 'real_time_factor', 1.0)
    for fn, name in [('physics', 'Physics'), ('user-commands', 'UserCommands'), ('scene-broadcaster', 'SceneBroadcaster'), ('contact', 'Contact')]:
        plugin(w, fn, name)
    plugin(w, 'imu', 'Imu')
    plugin(w, 'sensors', 'Sensors', [('render_engine', 'ogre2')])
    scene = tag(w, 'scene')
    tag(scene, 'ambient', '0.65 0.65 0.65 1')
    tag(scene, 'background', '0.12 0.16 0.22 1')
    tag(scene, 'shadows', 'true')
    light = tag(w, 'light', name='sun', type='directional')
    tag(light, 'pose', '0 0 12 0 0 0')
    tag(light, 'diffuse', '0.9 0.9 0.85 1')
    tag(light, 'specular', '0.2 0.2 0.2 1')
    tag(light, 'direction', '-0.4 0.2 -0.9')
    tag(light, 'cast_shadows', 'true')

    gui = tag(w, 'gui', fullscreen='false')
    p = tag(gui, 'plugin', filename='MinimalScene', name='3D View')
    tag(p, 'engine', 'ogre2')
    tag(p, 'scene', 'scene')
    tag(p, 'camera_pose', '10 -13 14 0 0.73 2.22')
    ui = tag(p, 'gz-gui')
    tag(ui, 'property', 'docked', type='string', key='state')
    for filename in ['GzSceneManager', 'InteractiveViewControl', 'CameraTracking', 'EntityContextMenuPlugin', 'WorldControl', 'WorldStats']:
        p = tag(gui, 'plugin', filename=filename, name=filename)
        if filename == 'WorldControl':
            tag(p, 'play_pause', 'true')
            tag(p, 'start_paused', 'false')
        if filename == 'WorldStats':
            for n in ['sim_time', 'real_time', 'real_time_factor']:
                tag(p, n, 'true')

    ground = tag(w, 'model', name='ground')
    tag(ground, 'static', 'true')
    l = tag(ground, 'link', name='ground')
    shape(l, 'ground_collision', 'box', '30 30 0.1', '0 0 -0.05 0 0 0')
    shape(l, 'ground_visual', 'box', '30 30 0.1', '0 0 -0.05 0 0 0', '0.105 0.13 0.17 1')
    island = tag(w, 'model', name='island')
    tag(island, 'static', 'true')
    l = tag(island, 'link', name='island')
    shape(l, 'grass', 'cylinder', (2.88, 0.02), '0 0 0.005 0 0 0', '0.12 0.34 0.24 1')
    # Tangent boxes overlap at their ends; no openings exist between wall segments.
    # 円周を接線方向の短い箱で囲む。箱の端を重ねて壁の隙間をなくす。
    for label, radius in [('inner', 3.0), ('outer', 6.0)]:
        m = tag(w, 'model', name=label + '_wall')
        tag(m, 'static', 'true')
        l = tag(m, 'link', name='wall')
        for j in range(72):
            a = j * math.tau / 72
            length = 2 * radius * math.tan(math.pi / 72) + 0.025
            pose = f'{radius*math.cos(a)} {radius*math.sin(a)} 0.45 0 0 {a+math.pi/2}'
            dims = f'{length} 0.16 0.9'
            shape(l, f'wall_{j}', 'box', dims, pose)
            color = ('0.08 0.55 0.68 1' if label == 'inner' else '0.84 0.88 0.92 1') if j % 6 else '0.98 0.57 0.16 1'
            shape(l, f'wall_visual_{j}', 'box', dims, pose, color)
    marks = tag(w, 'model', name='route_markings')
    tag(marks, 'static', 'true')
    l = tag(marks, 'link', name='paint')
    for j in range(60):
        a = j * math.tau / 60
        shape(l, f'dash_{j}', 'box', '0.22 0.055 0.006', f'{4.5*math.cos(a)} {4.5*math.sin(a)} 0.008 0 0 {a+math.pi/2}', '0.9 0.75 0.28 1')
    for row in range(2):
        for col in range(10):
            shape(l, f'start_{row}_{col}', 'box', '0.27 0.18 0.008', f'{3.2+col*0.27} {-0.25+row*0.18} 0.009 0 0 0', '0.95 0.95 0.95 1' if (row+col)%2 else '0.025 0.03 0.04 1')

    # ここから台車本体を作る。車体、車輪、関節、センサーを同じモデルへまとめる。
    m = tag(w, 'model', name='loop_cart')
    tag(m, 'pose', f'4.5 0 0.18 0 0 {math.pi/2}')
    l = tag(m, 'link', name='base_link')
    inertia(l, 12, 0.45, 0.95, 1.2)
    shape(l, 'body_collision', 'box', '0.9 0.55 0.22', '0 0 0.13 0 0 0')
    shape(l, 'body_visual', 'box', '0.9 0.55 0.22', '0 0 0.13 0 0 0', '0.04 0.48 0.82 1')
    shape(l, 'deck', 'box', '0.72 0.5 0.07', '-0.04 0 0.27 0 0 0', '0.72 0.81 0.88 1')
    shape(l, 'front_stripe', 'box', '0.04 0.5 0.04', '0.46 0 0.15 0 0 0', '0.3 1 0.6 1')
    shape(l, 'lidar_visual', 'cylinder', (0.085, 0.09), '0 0 0.39 0 0 0', '0.12 0.14 0.17 1')
    contact(l, 'body_collision', 'body')
    # 車輪の滑りによる向きの誤差を補う姿勢センサー。正解位置トピックとは独立。
    imu=tag(l,'sensor',name='imu',type='imu')
    tag(imu,'always_on','true');tag(imu,'update_rate',100)
    tag(imu,'topic','/loop/imu');tag(imu,'gz_frame_id','loop_cart/base_link')
    tag(imu,'imu')
    s = tag(l, 'sensor', name='lidar', type='gpu_lidar')
    tag(s, 'pose', '0 0 0.44 0 0 0')
    tag(s, 'topic', '/loop/scan')
    tag(s, 'gz_frame_id', 'loop_cart/lidar')
    tag(s, 'update_rate', 20)
    tag(s, 'always_on', 'true')
    tag(s, 'visualize', 'false')
    ray = tag(s, 'lidar')
    h = tag(tag(ray, 'scan'), 'horizontal')
    for k, v in [('samples', 720), ('resolution', 1), ('min_angle', -math.pi), ('max_angle', math.pi)]:
        tag(h, k, v)
    rg = tag(ray, 'range')
    for k, v in [('min', 0.06), ('max', 35), ('resolution', 0.005)]:
        tag(rg, k, v)
    # 左右それぞれ前後2輪を追加する。4輪は左右の速度差で旋回する構成。
    for side, y in [('left', .34), ('right', -.34)]:
        for end, x in [('front', .28), ('rear', -.28)]:
            name = end + '_' + side
            l = tag(m, 'link', name=name)
            tag(l, 'pose', f'{x} {y} 0 {-math.pi/2} 0 0')
            inertia(l, 0.7, .005, .005, .009)
            c = shape(l, name + '_collision', 'cylinder', (.18, .10))
            ode = tag(tag(tag(c, 'surface'), 'friction'), 'ode')
            tag(ode, 'mu', 1.0)
            tag(ode, 'mu2', 0.08)
            tag(ode, 'fdir1', '1 0 0')
            shape(l, name + '_tire', 'cylinder', (.18, .10), color='0.025 0.03 0.04 1')
            shape(l, name + '_hub', 'cylinder', (.085, .108), color='0.65 0.72 0.8 1')
            contact(l, name + '_collision', name)
            joint = tag(m, 'joint', name=name + '_joint', type='revolute')
            tag(joint, 'parent', 'base_link')
            tag(joint, 'child', name)
            ax = tag(joint, 'axis')
            tag(ax, 'xyz', '0 1 0', expressed_in='__model__')
            lim = tag(ax, 'limit')
            tag(lim, 'lower', -1e16)
            tag(lim, 'upper', 1e16)
            tag(lim, 'effort', 30)
            tag(lim, 'velocity', 30)
    # ROSから届く最終Twistを車輪回転へ変換する。Python側とは別に物理モデルの速度・加速度を制限する。
    plugin(m, 'diff-drive', 'DiffDrive', [
        ('left_joint', 'front_left_joint'), ('left_joint', 'rear_left_joint'),
        ('right_joint', 'front_right_joint'), ('right_joint', 'rear_right_joint'),
        ('wheel_separation', .68), ('wheel_radius', .18),
        ('topic', '/loop/cmd_vel'), ('odom_topic', '/loop/wheel_odom'),
        ('tf_topic', '/loop/wheel_tf'), ('frame_id', 'wheel_odom'), ('child_frame_id', 'loop_cart/base_link'),
        ('max_linear_velocity', 1.5), ('min_linear_velocity', -.8),
        ('max_angular_velocity', 1.5), ('min_angular_velocity', -1.5),
        ('max_linear_acceleration', 1.2), ('min_linear_acceleration', -2.5),
        ('max_angular_acceleration', 2.5), ('min_angular_acceleration', -2.5)])
    plugin(m, 'odometry-publisher', 'OdometryPublisher', [
        ('odom_topic', '/loop/ground_truth'), ('tf_topic', '/loop/tf'),
        ('odom_frame', 'world'), ('robot_base_frame', 'loop_cart/base_link'),
        ('dimensions', 3), ('odom_publish_frequency', 30)])
    overview = tag(w, 'model', name='overview_camera')
    tag(overview, 'static', 'true')
    link = tag(overview, 'link', name='camera_link')
    tag(link, 'pose', '9 -12 15 0 0.7854 2.2143')
    sensor = tag(link, 'sensor', name='overview', type='camera')
    tag(sensor, 'topic', '/loop/overview')
    tag(sensor, 'update_rate', 1)
    tag(sensor, 'always_on', 'true')
    camera = tag(sensor, 'camera')
    tag(camera, 'horizontal_fov', 1.05)
    img = tag(camera, 'image')
    tag(img, 'width', 1280)
    tag(img, 'height', 900)
    tag(img, 'format', 'R8G8B8')
    clip = tag(camera, 'clip')
    tag(clip, 'near', 0.1)
    tag(clip, 'far', 60)
    localization_landmarks(w,6.,6.)
    E.indent(sdf)
    out = ROOT / 'worlds' / 'loop_course.sdf'
    out.parent.mkdir(exist_ok=True)
    E.ElementTree(sdf).write(out, encoding='utf-8', xml_declaration=True)
    print(out)


if __name__ == '__main__':
    main()
