#!/usr/bin/env python3
"""Measure real SLAM maps and, optionally, explore the seven courses.

SDF geometry is an external test oracle: it defines the denominator and fixed
inspection stations. It is never converted into an occupancy map or passed to
the controller. During exploration, every intermediate goal is checked against
the latest SLAM occupancy grid; unknown cells keep their normal blocked state.
The initial spawn fixes the map-coordinate convention. Only an independent
external observer subscribes to ground truth to record validation errors;
control, goal selection and live position estimation never receive that input.
"""
import argparse
import hashlib
import json
import math
import os
import shutil
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import label, distance_transform_edt
from scipy.spatial import ConvexHull
import yaml

from world_clearance import WorldClearance

ROOT = Path(__file__).resolve().parents[1]
COURSES = (
    'loop_course', 'obstacle_course', 'oval_course', 'warehouse_course',
    'large_oval_course', 'large_warehouse_course', 'slalom_course',
)
ROBOT_RADIUS = .62
PLANNER_MARGIN = .28
SAMPLE_RESOLUTION = .10


def connected(mask, start):
    """Use four neighbours, so a diagonal corner cannot connect two rooms."""
    labels, count = label(mask)
    if not (0 <= start[0] < mask.shape[0] and 0 <= start[1] < mask.shape[1]):
        return np.zeros_like(mask), count
    component = labels[start]
    return labels == component if component else np.zeros_like(mask), count


def spawn(world):
    values = list(map(float, ET.parse(world).find(".//model[@name='loop_cart']/pose").text.split()))
    return values[0], values[1], values[5]


def world_to_map(x, y, initial):
    sx, sy, yaw = initial
    dx, dy = np.asarray(x)-sx, np.asarray(y)-sy
    return dx*math.cos(yaw)+dy*math.sin(yaw), -dx*math.sin(yaw)+dy*math.cos(yaw)


def reference_area(world):
    """Sample only the closed outer-wall enclosure, not the outside plane."""
    geometry = WorldClearance(world)
    outer = WorldClearance(world, walls_only=True)
    # Inner walls also exist in the ring courses. The outer envelope encloses
    # them; this reports disconnected inner islands as separate safe components.
    extents, wall_vertices = [], []
    for x, y, a, hx, hy, kind in outer.shapes:
        rx = hx if kind == 'circle' else abs(hx*math.cos(a))+abs(hy*math.sin(a))
        ry = hx if kind == 'circle' else abs(hx*math.sin(a))+abs(hy*math.cos(a))
        extents.append((x-rx, x+rx, y-ry, y+ry))
        if kind == 'circle':
            wall_vertices.extend((x+hx*math.cos(t), y+hx*math.sin(t))
                                 for t in np.linspace(0, math.tau, 48, endpoint=False))
        else:
            wall_vertices.extend((x+u*math.cos(a)-v*math.sin(a), y+u*math.sin(a)+v*math.cos(a))
                                 for u in (-hx, hx) for v in (-hy, hy))
    if not extents:
        raise ValueError('No enclosing walls: '+str(world))
    step = SAMPLE_RESOLUTION
    xs = np.arange(min(e[0] for e in extents), max(e[1] for e in extents)+step/2, step)
    ys = np.arange(min(e[2] for e in extents), max(e[3] for e in extents)+step/2, step)
    xx, yy = np.meshgrid(xs, ys)
    # All bundled outer enclosures are convex. Their wall-vertex hull excludes
    # the exterior corners of a ring's bounding rectangle; those corners must
    # not inflate the denominator. Inner islands remain explicitly reported.
    hull = ConvexHull(np.asarray(wall_vertices))
    enclosed = np.ones(xx.shape, dtype=bool)
    for ax, ay, offset in hull.equations:
        enclosed &= ax*xx+ay*yy+offset <= 1e-9
    safe = enclosed & (geometry.clearances(xx, yy) >= PLANNER_MARGIN)
    sx, sy, _ = spawn(world)
    start = int(np.argmin(abs(ys-sy))), int(np.argmin(abs(xs-sx)))
    reachable, components = connected(safe, start)
    if not reachable.any():
        raise ValueError('Spawn is outside the reference safe grid: '+str(world))
    return dict(xs=xs, ys=ys, xx=xx, yy=yy, safe=safe, reachable=reachable,
                components=components, geometry=geometry)


def read_saved_map(folder):
    config = yaml.safe_load((folder/'map.yaml').read_text())
    if abs(config['origin'][2]) > 1e-6 or config.get('mode', 'trinary') != 'trinary':
        raise ValueError('Coverage reader requires an unrotated trinary map')
    image = Path(config['image'])
    if not image.is_absolute():
        image = folder/image
    pixels = np.asarray(Image.open(image).convert('L'), dtype=np.float64)[::-1]
    probability = pixels/255 if config.get('negate', 0) else 1-pixels/255
    free = probability < config['free_thresh']
    occupied = probability > config['occupied_thresh']
    cells = np.full(free.shape, -1, dtype=np.int8)
    cells[free], cells[occupied] = 0, 100
    return cells, float(config['resolution']), tuple(config['origin'][:2])


def sample_map(cells, resolution, origin, x, y):
    """Unknown and out-of-image samples are never classified as observed."""
    x, y = np.broadcast_arrays(x, y)
    col = np.floor((x-origin[0])/resolution).astype(int)
    row = np.floor((y-origin[1])/resolution).astype(int)
    inside = (row >= 0) & (col >= 0) & (row < cells.shape[0]) & (col < cells.shape[1])
    known_free = (cells >= 0) & (cells < 50)
    clearance = distance_transform_edt(np.pad(known_free, 1))[1:-1, 1:-1]*resolution
    clearance -= ROBOT_RADIUS+resolution*.71
    observed = np.zeros(x.shape, dtype=bool)
    usable = np.zeros(x.shape, dtype=bool)
    observed[inside] = known_free[row[inside], col[inside]]
    usable[inside] = clearance[row[inside], col[inside]] >= PLANNER_MARGIN
    return observed, usable


def stations(world, reference):
    """Fixed external inspection route: ring, warehouse aisles, or slalom."""
    course = world.stem
    if course in ('loop_course', 'obstacle_course', 'oval_course', 'large_oval_course'):
        axes = {'loop_course': (4.5, 4.5), 'obstacle_course': (4.5, 4.5),
                'oval_course': (6., 4.5), 'large_oval_course': (10., 6.)}[course]
        proposed = [(axes[0]*math.cos(a), axes[1]*math.sin(a))
                    for a in np.linspace(0, math.tau, 17)]
    elif course == 'warehouse_course':
        proposed = [(0, -3.4), (4.8, -3.4), (4.8, 0), (4.8, 3.4),
                    (0, 3.4), (-4.8, 3.4), (-4.8, 0), (-4.8, -3.4),
                    (0, -3.4), (0, 0), (0, 3.4), (0, 0), (0, -3.4)]
    elif course == 'large_warehouse_course':
        proposed = [(-10, -7.5), (10, -7.5), (10, -2.5), (-10, -2.5),
                    (-10, 2.5), (10, 2.5), (10, 7.5), (-10, 7.5), (-10, -7.5)]
    else:
        proposed = [(-10, -5), (-9, 5), (-3, 5), (-3, -5), (3, -5),
                    (3, 5), (10, 5), (10, -5), (10, 5), (3, 5),
                    (3, -5), (-3, -5), (-3, 5), (-9, 5), (-10, -5)]
    # Stations themselves need the normal goal margin (.40); a path between
    # them may use the normal .28 corridor margin. Move a proposed station only
    # within 1 m, rather than silently selecting another part of the course.
    geometry = reference['geometry']
    candidates = reference['reachable'] & (geometry.clearances(reference['xx'], reference['yy']) >= .42)
    cx, cy = reference['xx'][candidates], reference['yy'][candidates]
    result = []
    for x, y in proposed:
        if geometry.clearance(x, y) < .42:
            distances = np.hypot(cx-x, cy-y)
            best = int(np.argmin(distances))
            if distances[best] > 1.:
                raise ValueError(f'No safe inspection station near {(x, y)} in {course}')
            x, y = float(cx[best]), float(cy[best])
        mx, my = world_to_map(x, y, spawn(world))
        result.append(dict(world_xy=[x, y], map_xy=[round(float(mx), 4), round(float(my), 4)]))
    return result


def measure(world):
    reference = reference_area(world)
    cells, resolution, origin = read_saved_map(ROOT/'maps'/world.stem)
    mx, my = world_to_map(reference['xx'], reference['yy'], spawn(world))
    observed, usable = sample_map(cells, resolution, origin, mx, my)
    areas = {}
    for name, mask in [('spawn_connected', reference['reachable']), ('all_enclosed_safe', reference['safe'])]:
        denominator = int(mask.sum())
        areas[name] = dict(samples=denominator, area_m2=round(denominator*SAMPLE_RESOLUTION**2, 3),
                           observed_free_fraction=round(float(observed[mask].mean()), 6),
                           usable_fraction=round(float(usable[mask].mean()), 6))
    metadata = json.loads((ROOT/'maps'/world.stem/'map_metadata.json').read_text())
    return dict(course=world.name, grid_sample_resolution_m=SAMPLE_RESOLUTION,
                robot_radius_m=ROBOT_RADIUS, safety_margin_m=PLANNER_MARGIN,
                reference_components=reference['components'], areas=areas,
                map_matches_world=metadata.get('world_sha256') == hashlib.sha256(world.read_bytes()).hexdigest(),
                saved_map_source=metadata.get('source'), inspection_stations=stations(world, reference))


def saved_route(world, inspection_stations):
    """Recheck every route station in the final measured map after loop closure.

    SLAM can optimize coordinates during a tour. A previously accepted target
    is therefore not automatically a valid target in the final saved grid.
    Route points may move by at most .75 m within the observed corridor; every
    connecting segment must pass the unmodified production A* planner.
    """
    from types import SimpleNamespace as Namespace
    from occupancy_navigation import OccupancyNavigationMap
    cells, resolution, origin = read_saved_map(ROOT/'maps'/world.stem)
    info = Namespace(resolution=resolution, width=cells.shape[1], height=cells.shape[0],
                     origin=Namespace(position=Namespace(x=origin[0], y=origin[1]),
                                      orientation=Namespace(z=0.)))
    grid = OccupancyNavigationMap(Namespace(info=info, data=cells.reshape(-1)))
    current = (0., 0.)
    component, _ = connected(grid.free, grid.cell(current))
    xx, yy = np.meshgrid(grid.xs, grid.ys)
    usable = component & (grid.static_clearances >= .42)
    points = np.column_stack((xx[usable], yy[usable]))
    if not len(points):
        raise AssertionError('Final saved map has no safe connected route stations')
    route, validations = [], []
    for station in inspection_stations[1:]:
        target = station['map_xy']
        distances = np.linalg.norm(points-np.asarray(target), axis=1)
        accepted = None
        for index in np.argsort(distances)[:120]:
            if distances[index] > .75:
                break
            candidate = tuple(round(float(v), 4) for v in points[index])
            try:
                path = grid.plan(current, candidate)
                accepted = dict(x=candidate[0], y=candidate[1], wait=2.)
                validations.append(dict(reference_station=target, saved_map_goal=list(candidate),
                                        displacement_m=float(distances[index]), path_points=len(path),
                                        clearance_m=grid.geometry.clearance(*candidate)))
                current = candidate
                break
            except ValueError:
                pass
        if accepted is None:
            raise AssertionError('Final map cannot connect inspection station '+str(target))
        route.append(accepted)
    return dict(version=1, world=world.name, frame='map', points=route), validations


def backup_artifacts(world):
    """Keep the prior map and route until this course has also exited cleanly."""
    folder = ROOT/'maps'/world.stem
    route = ROOT/'routes'/(world.stem+'_coverage_delivery.json')
    if folder.resolve().parent != (ROOT/'maps').resolve() or route.resolve().parent != (ROOT/'routes').resolve():
        raise ValueError('Artifact backup must stay inside the project')
    backup = ROOT/'logs/map_coverage_backups'/(str(time.time_ns())+'_'+world.stem)
    backup.mkdir(parents=True)
    existed = folder.is_dir()
    if existed:
        shutil.copytree(folder, backup/'map')
    route_existed = route.is_file()
    if route_existed:
        shutil.copy2(route, backup/'route.json')
    return dict(folder=folder, route=route, backup=backup, existed=existed, route_existed=route_existed)


def restore_artifacts(state):
    """Roll back this trial only, keeping already validated earlier courses."""
    folder, route, backup = state['folder'], state['route'], state['backup']
    if folder.resolve().parent != (ROOT/'maps').resolve() or route.resolve().parent != (ROOT/'routes').resolve():
        raise ValueError('Artifact rollback must stay inside the project')
    if state['existed']:
        restored = folder.with_name('.restore_'+folder.name+'_'+str(time.time_ns()))
        shutil.copytree(backup/'map', restored)
        if folder.exists():
            folder.rename(backup/'rejected_map')
        restored.rename(folder)
    elif folder.exists():
        folder.rename(backup/'rejected_map')
    if state['route_existed']:
        restored_route = route.with_name('.restore_'+route.name+'_'+str(time.time_ns()))
        shutil.copy2(backup/'route.json', restored_route)
        restored_route.replace(route)
    elif route.exists():
        route.rename(backup/'rejected_route.json')


def all_selected_passed(selected, results, cleanup_complete):
    return bool(cleanup_complete and len(results) == len(selected)
                and [r['course'] for r in results] == [c+'.sdf' for c in selected]
                and all(r.get('passed') for r in results))


def artifact_manifest(world):
    folder = ROOT/'maps'/world.stem
    config = yaml.safe_load((folder/'map.yaml').read_text())
    metadata = json.loads((folder/'map_metadata.json').read_text())
    image = Path(config['image'])
    if not image.is_absolute():
        image = folder/image
    route = ROOT/'routes'/(world.stem+'_coverage_delivery.json')
    files = [world, folder/'map.yaml', image, folder/'map_metadata.json', route]
    return dict(course=world.name, world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(),
                map_source=metadata.get('source'), map_frame=metadata.get('frame'),
                saved_at=metadata.get('saved_at'),
                files=[dict(path=str(path.relative_to(ROOT)), sha256=hashlib.sha256(path.read_bytes()).hexdigest())
                       for path in files])


def explore(selected, args):
    """Explore with live SLAM goals, then save actual LiDAR observations."""
    os.environ.setdefault('ROS_DOMAIN_ID', '42')
    os.environ.setdefault('GZ_PARTITION', 'ros2_gazebo_loop')
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from PySide2 import QtWidgets
    from course_selector import CoursePanel, PanelNode, COURSES as UI_COURSES, configure_japanese_font
    from occupancy_navigation import OccupancyNavigationMap
    from save_slam_map import save
    import psutil
    import fcntl
    from validation_processes import remaining_owned

    (ROOT/'logs').mkdir(exist_ok=True)
    lock = (ROOT/'logs/course_selector.lock').open('w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)

    app = QtWidgets.QApplication([])
    configure_japanese_font(app)
    rclpy.init()
    node = PanelNode()
    # A separate evaluator subscribes to simulation truth. Its values are used
    # only for recorded errors, never for selecting goals or generating motion.
    observer = rclpy.create_node('map_coverage_truth_observer')
    truth = [None]
    observer.create_subscription(Odometry, '/loop/ground_truth',
                                 lambda msg: truth.__setitem__(0, msg), qos_profile_sensor_data)
    panel = CoursePanel(node, simulation_gui=False)
    panel.show()
    results = []
    world = None
    pending_backup = None
    launch_handle = None
    log_offset = 0
    failure = None
    cleanup = dict(passed=False, completed=False)

    def write_results(completed=False, error=None):
        args.output.parent.mkdir(parents=True, exist_ok=True)
        data = dict(mode='explore', expected_courses=[c+'.sdf' for c in selected],
                    minimum_observed_free_fraction=args.minimum_coverage, results=results,
                    cleanup=cleanup, all_passed=all_selected_passed(selected, results, completed))
        if error is not None:
            data['failure'] = error
        args.output.write_text(json.dumps(data, indent=2))

    def stop_owned(handle=None, offset=None):
        """Reap CLI requests and the launch, including the 40/45 s fallback."""
        handle = handle or panel.process
        descendants = []
        if handle is not None:
            try:
                descendants = [(p.pid, p.create_time()) for p in psutil.Process(handle.pid).children(recursive=True)]
            except psutil.Error:
                pass
        began = time.monotonic()
        panel.stop_course()
        end = began+85.
        while (panel.process is not None or panel.shutdown_request is not None) and time.monotonic() < end:
            app.processEvents()
            rclpy.spin_once(observer, timeout_sec=0)
            time.sleep(.02)
        timed_out = panel.process is not None or panel.shutdown_request is not None
        terminal_errors = []
        if timed_out:
            # A failed test still owns its subprocesses. Reap them explicitly
            # rather than returning exit 0 with a Gazebo process left behind.
            if panel.shutdown_request is not None:
                try:
                    panel.shutdown_request.kill()
                    panel.shutdown_request.wait(timeout=5)
                except Exception as exc:
                    terminal_errors.append(str(exc))
            for pid, created in reversed(descendants):
                try:
                    process = psutil.Process(pid)
                    if process.create_time() == created and process.status() != psutil.STATUS_ZOMBIE:
                        process.kill()
                except psutil.Error:
                    pass
            if handle is not None and handle.poll() is None:
                try:
                    handle.kill()
                    handle.wait(timeout=5)
                except Exception as exc:
                    terminal_errors.append(str(exc))
            final_end = time.monotonic()+5
            while (panel.process is not None or panel.shutdown_request is not None) and time.monotonic() < final_end:
                app.processEvents()
                time.sleep(.02)
        # 起動待機中にsnapshot後から生まれた子も、同じ所有launch PGIDで検出する。
        # 別sessionへ移った捕捉済み子はPID+生成時刻で照合し続ける。
        identities = dict(descendants)
        remaining = remaining_owned(handle.pid, identities) if handle is not None else []
        lingering = list(remaining)
        if lingering:
            survivors = []
            for pid in lingering:
                try:
                    process = psutil.Process(pid)
                    known_child = pid in identities and process.create_time() == identities[pid]
                    same_group = handle is not None and os.getpgid(pid) == handle.pid
                    if known_child or same_group:
                        process.kill()
                        survivors.append(process)
                except (psutil.Error, ProcessLookupError, PermissionError):
                    pass
            psutil.wait_procs(survivors, timeout=5)
            remaining = remaining_owned(handle.pid, identities) if handle is not None else []
        events = list(panel.shutdown_events)
        forced = timed_out or bool(lingering) or any(name in ('SIGTERM', 'SIGKILL') for name, _ in events)
        code = handle.poll() if handle is not None else None
        log = ROOT/'logs/course_selector.log'
        with log.open('r', errors='replace') if log.exists() else open(os.devnull) as stream:
            if offset is not None:
                stream.seek(offset)
            block = stream.read() if offset is not None else ''
        failures = [marker for marker in ('process has died', 'failed to terminate', 'Segmentation fault',
                                           'Cannot shutdown', 'Traceback') if marker in block]
        completed = panel.process is None and panel.shutdown_request is None and not remaining
        return dict(completed=completed, elapsed_s=time.monotonic()-began, launch_exit_code=code,
                    shutdown_events=events, forced=forced, native_or_shutdown_errors=failures,
                    terminal_cleanup_errors=terminal_errors,
                    remaining_owned_pids=remaining,
                    passed=completed and not forced and not failures and not terminal_errors
                    and (handle is None or code == 0))

    write_results()

    def truth_error(world):
        if truth[0] is None:
            raise AssertionError('External ground-truth evaluator has no sample')
        position = truth[0].pose.pose.position
        tx, ty = world_to_map(position.x, position.y, spawn(world))
        pose = panel.navigation.data['pose']
        q = truth[0].pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))-spawn(world)[2]
        return dict(estimated_pose=pose, truth_map_xy=[float(tx), float(ty)],
                    position_error_m=math.hypot(pose[0]-tx, pose[1]-ty),
                    yaw_error_rad=abs(math.atan2(math.sin(pose[2]-yaw), math.cos(pose[2]-yaw))))

    def report():
        try:
            return json.loads((ROOT/'logs/loop_status.json').read_text())
        except (ValueError, OSError):
            return {}

    def until(predicate, timeout=60):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            app.processEvents()
            rclpy.spin_once(observer, timeout_sec=0)
            if report().get('collision_count', 0):
                raise AssertionError('Contact detected during map exploration')
            if predicate():
                return
            time.sleep(.02)
        raise TimeoutError(dict(course=panel.course_status.text(), navigation=panel.navigation.data))

    def pause(seconds):
        began = time.monotonic()
        until(lambda: time.monotonic()-began >= seconds, seconds+5)

    def go(target):
        panel.navigation.select(*target)
        target = (panel.navigation.x.value(), panel.navigation.y.value())
        panel.navigation.go()
        until(lambda: panel.navigation.data.get('goal') == list(target), 15)
        until(lambda: panel.navigation.data.get('state') in ('arrived', 'rejected', 'blocked'), args.goal_timeout)
        if panel.navigation.data['state'] != 'arrived':
            raise AssertionError(panel.navigation.data)
        pause(1.)

    def next_known_goal(target):
        grid = OccupancyNavigationMap(panel.navigation.last_grid)
        current = panel.navigation.data['pose'][:2]
        # Never plan with the SDF reference. A goal enters only a currently
        # observed, connected SLAM cell and passes the production A* check.
        component, _ = connected(grid.free, grid.cell(current))
        candidate_mask = component & (grid.static_clearances >= .42)
        xx, yy = np.meshgrid(grid.xs, grid.ys)
        distance = np.hypot(xx-current[0], yy-current[1])
        candidate_mask &= (distance >= .60) & (distance <= 3.)
        coordinates = np.column_stack((xx[candidate_mask], yy[candidate_mask]))
        if not len(coordinates):
            raise AssertionError('No connected observed goal with the normal safety margin')
        try:
            grid.plan(current, target)
            return tuple(target)
        except ValueError:
            pass
        order = np.argsort(np.linalg.norm(coordinates-np.asarray(target), axis=1))
        current_distance = math.dist(current, target)
        for candidate in coordinates[order[:120]]:
            if math.dist(candidate, target) >= current_distance-.25:
                continue
            try:
                grid.plan(current, candidate)
                return tuple(round(float(v), 4) for v in candidate)
            except ValueError:
                pass
        raise AssertionError('Observed frontier cannot safely progress towards station '+str(target))

    try:
        for course in selected:
            world = ROOT/'worlds'/(course+'.sdf')
            route = stations(world, reference_area(world))
            pending_backup = backup_artifacts(world)
            log = ROOT/'logs/course_selector.log'
            log_offset = log.stat().st_size if log.exists() else 0
            index = next(i for i, item in enumerate(UI_COURSES) if item[1] == world.name)
            panel.pose_source.setCurrentIndex(panel.pose_source.findData('slam'))
            truth[0] = None
            panel.combo.setCurrentIndex(index)
            panel.switch_course()
            until(lambda: panel.active_course == index and panel.active_pose_source == 'slam'
                  and not panel.starting and panel.stopping_since is None and panel.process is not None, 150)
            launch_handle = panel.process
            until(lambda: panel.navigation.last_grid is not None and bool(panel.navigation.data.get('pose')), 30)
            until(lambda: truth[0] is not None, 15)
            # One initial revolution observes all directions without position truth.
            previous = panel.navigation.data['pose'][2]
            angle, began = 0., time.monotonic()
            panel.turn_speed.set_value(.35)
            panel.held_buttons.add('left')
            while angle < 6.5 and time.monotonic()-began < 100:
                app.processEvents()
                rclpy.spin_once(observer, timeout_sec=0)
                yaw = panel.navigation.data['pose'][2]
                angle += abs(math.atan2(math.sin(yaw-previous), math.cos(yaw-previous)))
                previous = yaw
                time.sleep(.02)
            panel.clear_input()
            if angle < 6.5:
                raise AssertionError('Initial observation rotation did not complete')
            pause(3.)
            visits = []
            panel.auto_speed.set_value(args.mapping_speed)
            for number, station in enumerate(route):
                target, steps = station['map_xy'], []
                while math.dist(panel.navigation.data['pose'][:2], target) > .35:
                    if len(steps) >= 30:
                        raise AssertionError('Too many observed-frontier steps: '+str(station))
                    intermediate = next_known_goal(target)
                    print('GO', course, number, intermediate, 'station', target, flush=True)
                    go(intermediate)
                    steps.append(list(intermediate))
                visits.append(dict(station=station, observed_goals=steps, external_evaluation=truth_error(world)))
                progress = dict(course=world.name, completed_stations=visits, pose_source='slam',
                                ground_truth_used_for_control=False, mapping_speed_mps=args.mapping_speed)
                args.output.parent.mkdir(parents=True, exist_ok=True)
                args.output.with_suffix('.progress.json').write_text(json.dumps(progress, indent=2))
                print('VISITED', course, number, len(route),
                      'error_m', round(visits[-1]['external_evaluation']['position_error_m'], 4), flush=True)
            panel.set_mode(True)
            panel.stop()
            pause(2.)
            map_path = save(world)
            result = measure(world)
            result.update(stations_visited=visits, collision_count=report().get('collision_count'),
                          map_path=str(map_path), pose_source='slam', ground_truth_used_for_control=False,
                          external_endpoint_evaluation=truth_error(world), mapping_speed_mps=args.mapping_speed,
                          reached_all_stations=True)
            result['max_station_position_error_m'] = max(v['external_evaluation']['position_error_m'] for v in visits)
            result['max_station_yaw_error_rad'] = max(v['external_evaluation']['yaw_error_rad'] for v in visits)
            result['slam_parameters'] = yaml.safe_load((ROOT/'config/slam.yaml').read_text())['slam_toolbox']['ros__parameters']
            result['slam_parameters_sha256'] = hashlib.sha256(json.dumps(result['slam_parameters'], sort_keys=True).encode()).hexdigest()
            result['source_fingerprints'] = {name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
                                            for name in ('config/slam.yaml', 'config/ekf.yaml',
                                                         'scripts/loop_controller.py', 'scripts/dynamic_obstacles.py',
                                                         'scripts/occupancy_navigation.py', 'scripts/validate_map_coverage.py')}
            route_path = ROOT/'routes'/(course+'_coverage_delivery.json')
            route_path.parent.mkdir(parents=True, exist_ok=True)
            saved_points, validations = saved_route(world, route)
            route_path.write_text(json.dumps(saved_points, indent=2))
            result['route_path'] = str(route_path)
            result['final_map_route_validation'] = validations
            result['passed'] = (result['map_matches_world'] and result['collision_count'] == 0
                                and result['areas']['spawn_connected']['observed_free_fraction'] >= args.minimum_coverage
                                and result['external_endpoint_evaluation']['position_error_m'] < .35)
            if not result['passed']:
                raise AssertionError(result['areas'])
            cleanup = stop_owned(launch_handle, log_offset)
            result['shutdown'] = cleanup
            if not cleanup['passed']:
                raise AssertionError('Course cleanup did not finish normally: '+str(cleanup))
            result['artifact_manifest'] = artifact_manifest(world)
            results.append(result)
            pending_backup = None  # Commit this measured map only after clean exit.
            launch_handle = None
            write_results()
            print('PASS', course, result['areas'], flush=True)
    except BaseException as exc:
        # Preserve the measured grid for diagnosis; restore this course's prior
        # map and route after its owned processes have been stopped below.
        failure = dict(error_type=type(exc).__name__, error=str(exc),
                       navigation=panel.navigation.data, report=report(),
                       completed_results=results, ground_truth_used_for_control=False)
        if world is not None and truth[0] is not None and panel.navigation.data.get('pose'):
            failure['external_evaluation'] = truth_error(world)
        grid = panel.navigation.last_grid
        args.output.parent.mkdir(parents=True, exist_ok=True)
        if grid is not None:
            grid_path = args.output.with_suffix('.failure_map.npz')
            np.savez_compressed(grid_path, cells=np.asarray(grid.data, dtype=np.int8).reshape(grid.info.height, grid.info.width),
                                resolution=grid.info.resolution,
                                origin=[grid.info.origin.position.x, grid.info.origin.position.y])
            failure['measured_grid_snapshot'] = str(grid_path)
        args.output.with_suffix('.failure.json').write_text(json.dumps(failure, indent=2))
        failure['exception'] = exc
    finally:
        try:
            if panel.process is not None or panel.shutdown_request is not None or launch_handle is not None:
                cleanup = stop_owned(launch_handle, log_offset)
        except BaseException as exc:
            cleanup = dict(passed=False, completed=False, error=str(exc))
            if failure is None:
                failure = dict(error_type=type(exc).__name__, error=str(exc), exception=exc)
        try:
            if pending_backup is not None:
                restore_artifacts(pending_backup)
        except BaseException as exc:
            if failure is None:
                failure = dict(error_type=type(exc).__name__, error=str(exc), exception=exc)
            else:
                failure['rollback_error'] = str(exc)
        if not cleanup.get('passed') and failure is None:
            error = AssertionError('Final cleanup failed: '+str(cleanup))
            failure = dict(error_type=type(error).__name__, error=str(error), exception=error)
        failure = finalize_validation([
            ('panel_close', panel.close), ('panel_node', node.destroy_node),
            ('observer_node', observer.destroy_node), ('rclpy_shutdown', rclpy.try_shutdown),
            ('simulation_lock', lock.close),
        ], write_results, failure, cleanup.get('passed', False))
    if failure is not None:
        raise failure['exception']


def finalize_validation(cleanup_actions, write_results, failure, cleanup_passed):
    """最後のROS/ロック解放も完了してから合格を保存する。失敗時も残りを回収する。"""
    errors = []
    for name, action in cleanup_actions:
        try:
            action()
        except BaseException as exc:
            errors.append(dict(stage=name, error_type=type(exc).__name__, error=str(exc)))
            if failure is None:
                failure = dict(error_type=type(exc).__name__, error=str(exc), exception=exc)
    if errors:
        failure['final_cleanup_errors'] = errors
    public_failure = {key: value for key, value in failure.items() if key != 'exception'} if failure else None
    write_results(completed=failure is None and cleanup_passed, error=public_failure)
    return failure


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('courses', nargs='*', help='Course stems or .sdf filenames; omitted means all seven')
    parser.add_argument('--courses', nargs='+', dest='selected_courses', help='Course stems or .sdf filenames')
    parser.add_argument('--explore', action='store_true', help='Start owned SLAM simulations and update observed maps')
    parser.add_argument('--write-routes', action='store_true', help='Inspect and write routes checked against final saved maps')
    parser.add_argument('--minimum-coverage', type=float, default=.90,
                        help='Required observed-free fraction of spawn-connected safety samples (default 90%%)')
    parser.add_argument('--goal-timeout', type=float, default=180)
    parser.add_argument('--mapping-speed', type=float, default=.35,
                        help='SLAM tour forward-speed limit in m/s; default .35 (ordinary goal limit still applies)')
    parser.add_argument('--output', type=Path, default=ROOT/'logs/map_coverage_validation.json')
    args = parser.parse_args()
    if not 0 < args.minimum_coverage <= 1:
        parser.error('--minimum-coverage must be in (0, 1]')
    if not 0 < args.mapping_speed <= .55:
        parser.error('--mapping-speed must be in (0, .55]')
    selected = [Path(c).stem for c in (args.selected_courses or args.courses or COURSES)]
    if any(c not in COURSES for c in selected):
        parser.error('Unknown course; choose from '+', '.join(COURSES))
    if args.explore:
        explore(selected, args)
    else:
        results = [measure(ROOT/'worlds'/(course+'.sdf')) for course in selected]
        if args.write_routes:
            for result in results:
                course = Path(result['course']).stem
                route, validations = saved_route(ROOT/'worlds'/result['course'], result['inspection_stations'])
                path = ROOT/'routes'/(course+'_coverage_delivery.json')
                path.write_text(json.dumps(route, indent=2))
                result['final_map_route_validation'] = validations
                result['route_path'] = str(path)
        data = dict(mode='inspect', minimum_observed_free_fraction=args.minimum_coverage,
                    coverage_definition='Observed-free SLAM samples / SDF safe samples (radius .62m, margin .28m).',
                    disconnected_regions='Reported separately; inner islands cannot be reached from the spawn.',
                    results=results)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(data, indent=2))
        for result in results:
            print(result['course'], json.dumps(result['areas']), flush=True)


if __name__ == '__main__':
    main()
