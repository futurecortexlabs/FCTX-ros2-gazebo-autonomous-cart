#!/usr/bin/env python3
"""保存地図の配送を繰り返し、接触・鮮度・位置誤差・正常終了を監視する。"""
import argparse
import json
import math
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET

from validation_processes import owned_snapshot, verify_clean_stop

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
os.environ.setdefault('ROS_DOMAIN_ID', '42')
os.environ.setdefault('GZ_PARTITION', 'ros2_gazebo_loop')

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from PySide2 import QtWidgets
from course_selector import CoursePanel, PanelNode, COURSES, ROOT, configure_japanese_font
from mission_control import validate_route
from validate_saved_routes import source_manifest, FORBIDDEN_TRUTH_SUBSCRIBERS


def finalize_result(result, success, owned_cleanup, terminal_actions, save):
    """所有launchの正常終了と全ROS回収を確認した後にだけ合格を保存する。"""
    result['cleanup_completed'] = False
    owned_error = None
    try:
        owned_cleanup()
        result['cleanup_completed'] = True
    except BaseException as exc:
        # 配送完了直後のCtrl+C/SystemExitも、終了確認を省略した成功にしない。
        owned_error = exc
        result['cleanup_failure'] = f'{type(exc).__name__}: {exc}'
    errors = []
    for name, action in terminal_actions:
        try:
            action()
        except BaseException as exc:
            errors.append(dict(stage=name, error_type=type(exc).__name__, error=str(exc)))
    result['passed'] = bool(success and result['cleanup_completed'] and not errors
                            and not result.get('cleanup_failure'))
    if errors:
        result['terminal_cleanup_errors'] = errors
    save()
    if owned_error is not None:
        # SystemExit(0)からでも、失敗した検証器の終了コードは成功にしない。
        raise RuntimeError(result['cleanup_failure']) from owned_error
    if errors:
        raise RuntimeError(errors)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('minutes', nargs='?', type=float, default=10.)
    parser.add_argument('--route', type=Path, help='map座標の保存配送ルート。省略時は倉庫1.2m往復')
    args = parser.parse_args()
    if not math.isfinite(args.minutes) or args.minutes <= 0:
        parser.error('minutes must be finite and positive')
    if args.route:
        args.route = args.route.resolve()
    route = json.loads(args.route.read_text()) if args.route else dict(
        world='warehouse_course.sdf', frame='map',
        points=[dict(x=1.2, y=0., wait=3.), dict(x=0., y=0., wait=3.)])
    if route.get('frame') != 'map' or route.get('world') not in [c[1] for c in COURSES]:
        parser.error('route must use map coordinates and a bundled world')
    points = validate_route(route.get('points'))
    world = ROOT/'worlds'/route['world']
    spawn = list(map(float, ET.parse(world).find(".//model[@name='loop_cart']/pose").text.split()))
    manifest = source_manifest(world, args.route or ROOT/'routes/warehouse_course_localized_delivery.json')
    manifest['scripts/validate_endurance.py'] = __import__('hashlib').sha256(Path(__file__).read_bytes()).hexdigest()
    (ROOT/'logs').mkdir(exist_ok=True)
    outfile = ROOT/'logs/endurance_validation.json'
    logfile = ROOT/'logs/course_selector.log'
    offset = logfile.stat().st_size if logfile.exists() else 0
    result = dict(passed=False, cleanup_completed=False, requested_minutes=args.minutes, course=world.name,
                  route_points=points, source_files_sha256=manifest,
                  completed_cycles=0, cycle_endpoint_errors_m=[], gui_rss_kb=[],
                  max_status_age_seconds=0., max_estimation_error_m=0.)

    def save():
        outfile.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')

    app = QtWidgets.QApplication([])
    configure_japanese_font(app)
    rclpy.init()
    node = PanelNode()
    for subscription in list(node.subscriptions):
        if subscription.topic_name == '/loop/ground_truth':
            node.destroy_subscription(subscription)
    verifier = rclpy.create_node('endurance_external_verifier')
    truth = dict(message=None, received=0.)

    def receive_truth(message):
        truth.update(message=message, received=time.monotonic())

    verifier.create_subscription(Odometry, '/loop/ground_truth', receive_truth, qos_profile_sensor_data)
    panel = CoursePanel(node, simulation_gui=False)
    panel.show()
    began = None
    last_progress = 0.
    owned = None
    success = False
    report_signature = None
    report_observed = time.monotonic()

    def report():
        return json.loads((ROOT/'logs/loop_status.json').read_text())

    def position_error():
        p = truth['message'].pose.pose.position
        sx, sy, _, _, _, yaw = spawn
        x = (p.x-sx)*math.cos(yaw)+(p.y-sy)*math.sin(yaw)
        y = -(p.x-sx)*math.sin(yaw)+(p.y-sy)*math.cos(yaw)
        pose = panel.navigation.data['pose']
        return math.hypot(pose[0]-x, pose[1]-y)

    def until(predicate, seconds):
        nonlocal last_progress, report_signature, report_observed
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            app.processEvents()
            rclpy.spin_once(verifier, timeout_sec=0)
            now = time.monotonic()
            if began is not None:
                data = report()
                assert data['collision_count'] == 0, data
                assert data['course_world'] == str(world)
                age = now-panel.navigation.last_received
                result['max_status_age_seconds'] = max(result['max_status_age_seconds'], age)
                assert age < 2. and now-node.state_time < 2. and now-truth['received'] < 2., 'stale update'
                stat = (ROOT/'logs/loop_status.json').stat()
                signature = (stat.st_mtime_ns, stat.st_ino, stat.st_size)
                if signature != report_signature:
                    report_signature, report_observed = signature, now
                assert now-report_observed < 5., 'status report did not update'
                assert panel.process is not None and panel.process.poll() is None
                result['max_estimation_error_m'] = max(result['max_estimation_error_m'], position_error())
                if now-last_progress >= 60:
                    last_progress = now
                    result['elapsed_seconds'] = now-began
                    save()
                    print('PROGRESS', round(now-began), 'seconds; cycles', result['completed_cycles'],
                          'mission', panel.navigation.data.get('mission'), flush=True)
            if predicate():
                return
            time.sleep(.02)
        raise TimeoutError(panel.navigation.data)

    def settled(seconds):
        started = time.monotonic()
        until(lambda: time.monotonic()-started >= seconds, seconds+5)

    save()
    try:
        panel.pose_source.setCurrentIndex(panel.pose_source.findData('localization'))
        panel.combo.setCurrentIndex(next(i for i, c in enumerate(COURSES) if c[1] == world.name))
        panel.switch_course()
        owned = panel.process
        until(lambda: panel.process is not None and not panel.starting and panel.stopping_since is None, 150)
        until(lambda: truth['message'] is not None and time.monotonic()-truth['received'] < .5
              and time.monotonic()-panel.navigation.last_received < .5, 20)
        assert position_error() < .35, 'initial position mismatch'
        names = [info.node_name for info in verifier.get_subscriptions_info_by_topic('/loop/ground_truth')]
        assert set(names) == {verifier.get_name()}, names
        present = {name for name, _ in verifier.get_node_names_and_namespaces()}
        assert FORBIDDEN_TRUTH_SUBSCRIBERS <= present
        result['ground_truth_subscribers'] = names
        began = time.monotonic()
        while time.monotonic()-began < args.minutes*60:
            ui = panel.navigation.mission
            ui.table.setRowCount(0)
            if args.route:
                ui.read_route(args.route)
            else:
                for point in points:
                    panel.navigation.select(point['x'], point['y'])
                    ui.add()
            assert ui.points() == points
            ui.send('start')
            request_id = ui.pending['id']
            until(lambda: panel.navigation.data.get('mission_reply', {}).get('id') == request_id, 20)
            assert panel.navigation.data['mission_reply']['ok']
            until(lambda: panel.navigation.data.get('mission', {}).get('state') in ('completed', 'failed'),
                  max(180., len(points)*60.+sum(p['wait'] for p in points)))
            mission = panel.navigation.data['mission']
            assert mission['state'] == 'completed' and mission['index'] == mission['total'] == len(points), mission
            settled(2.)
            speed = truth['message'].twist.twist
            assert math.hypot(speed.linear.x, speed.linear.y) < .02 and abs(speed.angular.z) < .03
            error = position_error()
            assert error < .35, error
            result['cycle_endpoint_errors_m'].append(error)
            result['completed_cycles'] += 1
            result['gui_rss_kb'].append(int(next(line.split()[1] for line in Path('/proc/self/status').read_text().splitlines() if line.startswith('VmRSS:'))))
            result.update(elapsed_seconds=time.monotonic()-began, report=report())
            save()
            print('PASS cycle', result['completed_cycles'], 'seconds', round(result['elapsed_seconds']),
                  'endpoint error', round(error, 4), flush=True)
        assert result['elapsed_seconds'] >= args.minutes*60
        final_manifest = source_manifest(world, args.route or ROOT/'routes/warehouse_course_localized_delivery.json')
        assert all(final_manifest.get(k) == v for k, v in manifest.items()), 'Sources changed during endurance test'
        success = True
    except Exception as exc:
        result['failure'] = f'{type(exc).__name__}: {exc}'
        raise
    finally:
        began = None
        stopped = time.monotonic()

        def cleanup_owned():
            identities = owned_snapshot(owned) if owned is not None else {}
            panel.close()
            until(lambda: panel.process is None and panel.shutdown_request is None, 85)
            segment = logfile.read_bytes()[offset:].decode(errors='replace')
            assert owned is not None, 'No owned launch handle'
            remaining = verify_clean_stop(owned, panel.shutdown_events, identities, segment)
            result.update(shutdown_seconds=time.monotonic()-stopped, shutdown_events=panel.shutdown_events,
                          launch_exit_code=owned.returncode, remaining_owned_pids=remaining)

        finalize_result(result, success, cleanup_owned, [
            ('verifier', verifier.destroy_node), ('panel_node', node.destroy_node),
            ('rclpy', rclpy.try_shutdown),
        ], save)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
