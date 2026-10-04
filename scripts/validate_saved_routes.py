#!/usr/bin/env python3
"""拡張した保存地図でAMCLを起動し、保存済み配送ルートの全地点を巡回する。

制御へ正解位置を渡さず、外部検証ノードだけが正解位置を受信して最終誤差と
停止を測定する。ROS 2 setup.bashを読み込んで実行する。所有パネルの
シミュレーションだけを起動・終了し、失敗すると次のコースを開始しない。
"""
from __future__ import annotations

import argparse
from datetime import datetime, timedelta, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

from validation_processes import owned_snapshot, verify_clean_stop

ROOT = Path(__file__).resolve().parents[1]
COURSE_STEMS = (
    'loop_course', 'obstacle_course', 'oval_course', 'warehouse_course',
    'large_oval_course', 'large_warehouse_course', 'slalom_course',
)
JST = timezone(timedelta(hours=9))
FORBIDDEN_TRUTH_SUBSCRIBERS = {
    'loop_autonomous_driver', 'loop_safety_gate',
    'loop_localization_bridge', 'ekf_filter_node',
}


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def manifest_key(path):
    """外部ルートも内容を識別し、開発環境の絶対パスを公開記録へ出さない。"""
    path=Path(path).resolve()
    try:return path.relative_to(ROOT.resolve()).as_posix()
    except ValueError:return 'external/'+path.name


def source_manifest(world, route):
    # 制御器がimportするモジュールや起動シェルも記録し、依存だけの変更を見逃さない。
    paths=[Path(world),Path(route)]
    paths+=list((ROOT/'scripts').rglob('*.py'))
    paths+=list((ROOT/'launch').rglob('*.py'))
    paths+=[p for p in (ROOT/'config').rglob('*') if p.suffix in ('.yaml','.yml')]
    paths+=list(ROOT.glob('*.sh'))
    paths+=list((ROOT/'fonts').glob('*.conf'))
    paths+=list((ROOT/'maps'/Path(world).stem).glob('*'))
    return {manifest_key(path):sha256(path) for path in sorted(set(paths)) if path.is_file()}


def finalize_result(record, exit_code, owned_cleanup, terminal_actions, save):
    """終了中の中断も失敗として保存し、残りのROS回収を必ず試みる。"""
    candidate = record.get('all_passed', False) and exit_code == 0
    record['all_passed'] = False
    record['final_cleanup_completed'] = False
    owned_complete = False
    try:
        owned_cleanup()
        owned_complete = True
    except BaseException as exc:
        exit_code = 1
        record['cleanup_failure'] = f'{type(exc).__name__}: {exc}'
    errors = []
    for name, action in terminal_actions:
        try:
            action()
        except BaseException as exc:
            errors.append(dict(stage=name, error_type=type(exc).__name__, error=str(exc)))
    if errors:
        exit_code = 1
        record['terminal_cleanup_errors'] = errors
    record['final_cleanup_completed'] = owned_complete and not errors
    record['all_passed'] = bool(candidate and record['final_cleanup_completed'])
    record['finished_at_jst'] = datetime.now(JST).isoformat(timespec='seconds')
    save()
    return exit_code


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--courses', nargs='+', default=list(COURSE_STEMS),
                        help='Course stems or .sdf filenames; omitted means all seven')
    parser.add_argument('--route-suffix', default='coverage_delivery',
                        help='routes/<course>_<suffix>.json (default: coverage_delivery)')
    parser.add_argument('--output', type=Path, default=ROOT/'logs/saved_routes_validation.json')
    args = parser.parse_args()
    selected = [Path(name).stem for name in args.courses]
    if any(name not in COURSE_STEMS for name in selected) or len(set(selected)) != len(selected):
        parser.error('--courses must be distinct bundled course names')
    if not args.route_suffix or '/' in args.route_suffix or '\\' in args.route_suffix:
        parser.error('--route-suffix must be a filename suffix without a path')

    os.environ.setdefault('ROS_DOMAIN_ID', '42')
    os.environ.setdefault('GZ_PARTITION', 'ros2_gazebo_loop')
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from PyQt5 import QtWidgets
    from course_selector import CoursePanel, PanelNode, COURSES, configure_japanese_font
    from mission_control import validate_route

    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    (ROOT/'logs').mkdir(parents=True, exist_ok=True)
    record = dict(
        started_at_jst=datetime.now(JST).isoformat(timespec='seconds'),
        scope='AMCL restart with saved observed maps, saved map-frame full inspection routes, final error and stop',
        ros_domain_id=os.environ['ROS_DOMAIN_ID'], gz_partition=os.environ['GZ_PARTITION'],
        endpoint_error_limit_m=.35, results=[], all_passed=False, final_cleanup_completed=False,
    )

    def save():
        output.write_text(json.dumps(record, ensure_ascii=False, indent=2)+'\n')

    save()
    prepared = []
    # 未観測の地点や異なる座標系を含むルートを、起動前に形式とコースで照合する。
    try:
        for stem in selected:
            world = ROOT/'worlds'/(stem+'.sdf')
            route_path = ROOT/'routes'/(stem+'_'+args.route_suffix+'.json')
            route = json.loads(route_path.read_text())
            assert route.get('version') == 1 and route.get('world') == world.name and route.get('frame') == 'map', route
            points = validate_route(route.get('points'))
            metadata = json.loads((ROOT/'maps'/stem/'map_metadata.json').read_text())
            assert metadata['world_sha256'] == sha256(world), 'Saved map/world mismatch: '+stem
            spawn = list(map(float, ET.parse(world).find(".//model[@name='loop_cart']/pose").text.split()))
            prepared.append((world, route_path, points, spawn, source_manifest(world, route_path)))
    except Exception as exc:
        record['failure'] = f'Preflight {type(exc).__name__}: {exc}'
        save()
        print(record['failure'], file=sys.stderr, flush=True)
        return 1

    app = QtWidgets.QApplication([])
    configure_japanese_font(app)
    rclpy.init()
    node = PanelNode()
    # 操作パネルの速度表示用購読も外し、この試験では検証ノードだけに正解を渡す。
    removed = 0
    for subscription in list(node.subscriptions):
        if subscription.topic_name == '/loop/ground_truth':
            node.destroy_subscription(subscription)
            removed += 1
    verifier = rclpy.create_node('saved_routes_verifier')
    truth = {'message': None, 'received': 0.}

    def receive_truth(message):
        truth['message'], truth['received'] = message, time.monotonic()

    verifier.create_subscription(Odometry, '/loop/ground_truth', receive_truth, qos_profile_sensor_data)
    panel = CoursePanel(node, simulation_gui=False)
    panel.show()
    record['removed_panel_truth_subscriptions'] = removed
    active_case = None
    monitoring = False
    expected_world = None
    last_progress = time.monotonic()
    status_signature = None
    status_changed = 0.
    exit_code = 0

    def report():
        nonlocal status_signature,status_changed
        try:
            path=ROOT/'logs/loop_status.json'
            data=json.loads(path.read_text())
            info=path.stat()
            signature=(info.st_mtime_ns,info.st_ino,info.st_size)
            if data.get('course_world') == str(expected_world) and signature != status_signature:
                status_signature=signature
                status_changed=time.monotonic()
            return data
        except (OSError, ValueError):
            return {}

    def events():
        app.processEvents()
        rclpy.spin_once(verifier, timeout_sec=0)

    def check_runtime():
        now = time.monotonic()
        assert panel.process is not None and panel.process.poll() is None, 'Owned launch exited during delivery'
        age = now-panel.navigation.last_received
        active_case['max_navigation_age_seconds'] = max(active_case['max_navigation_age_seconds'], age)
        assert age < 2., ('Navigation update stale', age)
        assert now-node.state_time < 2., 'Safety update stale'
        assert now-truth['received'] < 2., 'External truth update stale'
        data = panel.navigation.data
        assert data.get('pose_source') == 'localization' and data.get('frame') == 'map', data
        latest = report()
        assert latest.get('course_world') == str(expected_world), latest
        assert latest.get('collision_count') == 0, ('Contact detected', latest)
        # 3秒周期のJSONは置換/更新を観測した単調時計で監視し、壁時計の差を使わない。
        status_age=time.monotonic()-status_changed
        active_case['max_status_update_age_seconds']=max(active_case['max_status_update_age_seconds'],status_age)
        assert status_changed and status_age < 5., ('Status file update stale',status_age)

    def until(predicate, timeout, label):
        nonlocal last_progress
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            events()
            if monitoring:
                check_runtime()
            if predicate():
                return
            if time.monotonic()-last_progress >= 60:
                print('PROGRESS', expected_world.name if expected_world else '', label,
                      json.dumps(dict(mission=panel.navigation.data.get('mission'),
                                      pose=panel.navigation.data.get('pose'),
                                      detail=panel.navigation.data.get('detail')), ensure_ascii=False), flush=True)
                last_progress = time.monotonic()
                save()
            time.sleep(.02)
        raise TimeoutError(dict(stage=label, course=panel.course_status.text(), navigation=panel.navigation.data))

    def pause(seconds):
        began = time.monotonic()
        until(lambda: time.monotonic()-began >= seconds, seconds+5., 'settle')

    def position_error(spawn):
        assert truth['message'] is not None and time.monotonic()-truth['received'] < .5
        assert time.monotonic()-panel.navigation.last_received < .5
        p = truth['message'].pose.pose.position
        sx, sy, _, _, _, yaw = spawn
        x = (p.x-sx)*math.cos(yaw)+(p.y-sy)*math.sin(yaw)
        y = -(p.x-sx)*math.sin(yaw)+(p.y-sy)*math.cos(yaw)
        estimate = panel.navigation.data['pose']
        assert all(math.isfinite(value) for value in (x, y, *estimate))
        return math.hypot(estimate[0]-x, estimate[1]-y), [x, y], list(estimate)

    def subscribers():
        return sorted((info.node_name, info.node_namespace) for info in verifier.get_subscriptions_info_by_topic('/loop/ground_truth'))

    try:
        for world, route_path, points, spawn, manifest in prepared:
            expected_world = world
            truth.update(message=None, received=0.)
            status_signature,status_changed=None,0.
            monitoring = False
            last_progress = time.monotonic()
            active_case = dict(course=world.name, route=manifest_key(route_path),
                               route_frame='map', point_count=len(points), source_files_sha256=manifest,
                               max_navigation_age_seconds=0., max_status_update_age_seconds=0., passed=False)
            record['results'].append(active_case)
            save()
            logfile = ROOT/'logs/course_selector.log'
            offset = logfile.stat().st_size if logfile.exists() else 0
            index = next(i for i, item in enumerate(COURSES) if item[1] == world.name)
            panel.pose_source.setCurrentIndex(panel.pose_source.findData('localization'))
            panel.combo.setCurrentIndex(index)
            panel.switch_course()
            until(lambda: panel.active_course == index and panel.active_pose_source == 'localization'
                  and panel.process is not None and not panel.starting and panel.stopping_since is None,
                  150., 'AMCL startup')
            until(lambda: bool(panel.navigation.data.get('pose')) and panel.navigation.data.get('frame') == 'map'
                  and panel.navigation.map.map_image is not None and truth['message'] is not None
                  and time.monotonic()-truth['received'] < .5
                  and time.monotonic()-panel.navigation.last_received < .5,
                  30., 'fresh initial pose and saved map')
            until(lambda: report().get('course_world') == str(world), 15., 'current-course initial report')
            assert node.mode == 'manual'
            initial_error, initial_truth, initial_estimate = position_error(spawn)
            assert initial_error < .35, ('Initial AMCL/world alignment', initial_error)
            expected_subscriber = (verifier.get_name(), verifier.get_namespace())
            until(lambda: expected_subscriber in subscribers(), 10., 'ROS graph discovery')
            names = subscribers()
            assert all(name not in FORBIDDEN_TRUTH_SUBSCRIBERS for name, _ in names), names
            assert set(names) == {expected_subscriber}, ('Truth is not limited to the external verifier', names)
            present = {name for name, _ in verifier.get_node_names_and_namespaces()}
            assert FORBIDDEN_TRUTH_SUBSCRIBERS <= present, ('Missing control/EKF nodes', present)
            monitoring = True
            pause(1.)
            actual = truth['message'].twist.twist
            assert math.hypot(actual.linear.x, actual.linear.y) < .02 and abs(actual.angular.z) < .03
            active_case.update(initial_error_m=initial_error, initial_truth_map_xy=initial_truth,
                               initial_estimated_pose=initial_estimate, ground_truth_subscribers=names,
                               startup_retries=panel.startup_retries)
            ui = panel.navigation.mission
            ui.read_route(route_path)
            assert ui.points() == points, 'UI did not load every saved waypoint'
            ui.send('start')
            assert ui.pending is not None, ui.status.text()
            request_id = ui.pending['id']
            until(lambda: panel.navigation.data.get('mission_reply', {}).get('id') == request_id,
                  20., 'mission acknowledgement')
            reply = panel.navigation.data['mission_reply']
            assert reply.get('ok'), reply
            began = time.monotonic()
            timeout = max(180., len(points)*60.+sum(point['wait'] for point in points))
            until(lambda: panel.navigation.data.get('mission', {}).get('state') in ('completed', 'failed'),
                  timeout, 'saved route full mission')
            mission = panel.navigation.data['mission']
            assert mission['state'] == 'completed' and mission['index'] == mission['total'] == len(points), mission
            pause(2.)
            actual = truth['message'].twist.twist
            assert math.hypot(actual.linear.x, actual.linear.y) < .02 and abs(actual.angular.z) < .03, actual
            error, truth_xy, estimate = position_error(spawn)
            assert error < .35, ('Final estimated/truth position difference', error)
            latest = report()
            active_case.update(
                elapsed_seconds=time.monotonic()-began, mission=mission,
                endpoint_error_m=error, endpoint_truth_map_xy=truth_xy, endpoint_estimated_pose=estimate,
                estimated_goal_distance_m=math.dist(estimate[:2], [points[-1]['x'], points[-1]['y']]),
                truth_goal_distance_m=math.dist(truth_xy, [points[-1]['x'], points[-1]['y']]),
                actual_speed_mps=math.hypot(actual.linear.x, actual.linear.y),
                actual_turn_rate_rps=actual.angular.z, collision_count=latest['collision_count'],
            )
            # 地図・ルート・制御ソースが試験途中に変わった場合は別の証拠として合格させない。
            assert source_manifest(world, route_path) == manifest, 'Sources or saved map changed during test'
            monitoring = False
            owned = panel.process
            identities = owned_snapshot(owned)
            stopped = time.monotonic()
            panel.stop_course()
            until(lambda: panel.process is None and panel.shutdown_request is None, 85., 'owned course shutdown')
            with logfile.open('rb') as log:
                log.seek(offset)
                segment = log.read().decode(errors='replace')
            remaining = verify_clean_stop(owned, panel.shutdown_events, identities, segment)
            active_case.update(shutdown_seconds=time.monotonic()-stopped,
                               launch_exit_code=owned.returncode, shutdown_acknowledged=panel.shutdown_acknowledged,
                               shutdown_events=panel.shutdown_events, remaining_owned_pids=remaining, passed=True)
            save()
            print('PASS AMCL saved route', world.name, len(points), 'points, contacts 0, endpoint error', round(error, 4), flush=True)
        record['all_passed'] = True
    except (Exception, KeyboardInterrupt) as exc:
        exit_code = 1
        record['failure'] = f'{type(exc).__name__}: {exc}'
        if active_case is not None:
            active_case['failure'] = record['failure']
        print(record['failure'], file=sys.stderr, flush=True)
    finally:
        monitoring = False

        def cleanup_owned():
            panel.close()
            until(lambda: panel.process is None and panel.shutdown_request is None, 85., 'final owned cleanup')

        exit_code = finalize_result(record, exit_code, cleanup_owned, [
            ('verifier', verifier.destroy_node), ('panel_node', node.destroy_node),
            ('rclpy', rclpy.try_shutdown),
        ], save)
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
