"""Coverage accounting must not count unknown/outside or disconnected space."""
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from PIL import Image
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import validate_map_coverage as coverage
from validate_map_coverage import (connected, sample_map, world_to_map, reference_area,
                                   saved_route, backup_artifacts, restore_artifacts, all_selected_passed)


class CoverageAccounting(unittest.TestCase):
    def test_corner_touch_and_invalid_spawn_are_not_connected(self):
        mask = np.eye(3, dtype=bool)
        reachable, count = connected(mask, (0, 0))
        self.assertEqual(count, 3)
        self.assertEqual(int(reachable.sum()), 1)
        self.assertFalse(connected(mask, (0, 1))[0].any())
        self.assertFalse(connected(mask, (3, 3))[0].any())

    def test_unknown_obstacle_and_outside_never_count_as_free(self):
        cells = np.zeros((30, 30), dtype=np.int8)
        cells[15, 15], cells[16, 16] = -1, 100
        observed, _ = sample_map(cells, .1, (-1.5, -1.5),
                                 np.array([-.25, .05, .15, 2]), np.array([-.25, .05, .15, 0]))
        self.assertEqual(observed.tolist(), [True, False, False, False])

    def test_map_border_and_unknown_keep_normal_robot_margin(self):
        cells = np.zeros((100, 100), dtype=np.int8)
        cells[:, 50] = -1
        observed, usable = sample_map(cells, .1, (0, 0),
                                      np.array([.15, 2.5, 4.8, 7.5]), np.array([5, 5, 5, 5]))
        self.assertTrue(observed.all())
        self.assertEqual(usable.tolist(), [False, True, False, True])

    def test_fixed_initial_transform_rotates_without_live_truth(self):
        mx, my = world_to_map(np.array([10, 8]), np.array([-5, -7]), (10, -7, math.pi/2))
        np.testing.assert_allclose(mx, [2, 0], atol=1e-12)
        np.testing.assert_allclose(my, [0, 2], atol=1e-12)

    def test_ring_exterior_is_excluded_and_inner_island_is_reported(self):
        reference = reference_area(Path(__file__).resolve().parents[1]/'worlds/loop_course.sdf')
        xs, ys = reference['xs'], reference['ys']
        exterior = (int(np.argmin(abs(ys-5.7))), int(np.argmin(abs(xs-5.7))))
        island = (int(np.argmin(abs(ys))), int(np.argmin(abs(xs))))
        self.assertFalse(reference['safe'][exterior])
        self.assertTrue(reference['safe'][island])
        self.assertFalse(reference['reachable'][island])

    def test_saved_route_cannot_cross_unknown_separator(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            folder = root/'maps'/'example'
            folder.mkdir(parents=True)
            pixels = np.full((100, 100), 254, dtype=np.uint8)
            pixels[:, 60] = 205  # Unobserved column splits the safe corridors.
            Image.fromarray(pixels).save(folder/'map.pgm')
            (folder/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm', resolution=.1,
                origin=[-5, -5, 0], free_thresh=.196, occupied_thresh=.65, negate=0, mode='trinary')))
            stations = [dict(map_xy=[0, 0]), dict(map_xy=[3, 0])]
            with patch.object(coverage, 'ROOT', root):
                with self.assertRaisesRegex(AssertionError, 'cannot connect'):
                    saved_route(root/'worlds/example.sdf', stations)
                pixels[:, 60] = 254
                Image.fromarray(pixels).save(folder/'map.pgm')
                route, checks = saved_route(root/'worlds/example.sdf', stations)
                self.assertEqual(len(route['points']), 1)
                self.assertGreaterEqual(checks[0]['clearance_m'], .42)

    def test_partial_or_unclean_exploration_never_reports_full_success(self):
        selected = ['loop_course', 'oval_course']
        first = dict(course='loop_course.sdf', passed=True)
        second = dict(course='oval_course.sdf', passed=True)
        self.assertFalse(all_selected_passed(selected, [first], True))
        self.assertFalse(all_selected_passed(selected, [first, second], False))
        self.assertTrue(all_selected_passed(selected, [first, second], True))

    def test_terminal_cleanup_failure_cannot_publish_success(self):
        events, saved = [], []
        def broken_ros_cleanup():
            events.append('ros')
            raise RuntimeError('injected ROS cleanup failure')
        def release_lock():
            events.append('lock')
        def write(**result):
            saved.append((events[:], result))
        failure = coverage.finalize_validation([
            ('ROS', broken_ros_cleanup), ('lock', release_lock),
        ], write, None, True)
        self.assertIsInstance(failure['exception'], RuntimeError)
        self.assertEqual(saved[0][0], ['ros', 'lock'])
        self.assertFalse(saved[0][1]['completed'])
        self.assertEqual(saved[0][1]['error']['final_cleanup_errors'][0]['stage'], 'ROS')

    def test_success_is_published_only_after_last_cleanup(self):
        events, saved = [], []
        failure = coverage.finalize_validation([
            ('ROS', lambda: events.append('ros')),
            ('lock', lambda: events.append('lock')),
        ], lambda **result: saved.append((events[:], result)), None, True)
        self.assertIsNone(failure)
        self.assertEqual(saved, [(['ros', 'lock'], {'completed': True, 'error': None})])

    def test_failed_trial_restores_existing_map_and_route(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            folder = root/'maps/loop_course'
            folder.mkdir(parents=True)
            (folder/'map.pgm').write_bytes(b'old measured map')
            route = root/'routes/loop_course_coverage_delivery.json'
            route.parent.mkdir()
            route.write_bytes(b'old route')
            with patch.object(coverage, 'ROOT', root):
                backup = backup_artifacts(root/'worlds/loop_course.sdf')
                (folder/'map.pgm').write_bytes(b'incomplete trial')
                (folder/'pending_map.yaml').write_bytes(b'incomplete temporary map')
                route.write_bytes(b'incomplete route')
                restore_artifacts(backup)
            self.assertEqual((folder/'map.pgm').read_bytes(), b'old measured map')
            self.assertFalse((folder/'pending_map.yaml').exists())
            self.assertEqual(route.read_bytes(), b'old route')
            self.assertEqual((backup['backup']/'rejected_map/map.pgm').read_bytes(), b'incomplete trial')

    def test_failed_first_trial_leaves_no_distributed_map(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/'maps').mkdir()
            (root/'routes').mkdir()
            with patch.object(coverage, 'ROOT', root):
                backup = backup_artifacts(root/'worlds/loop_course.sdf')
                folder = root/'maps/loop_course'
                folder.mkdir()
                (folder/'map.pgm').write_bytes(b'incomplete trial')
                restore_artifacts(backup)
            self.assertFalse(folder.exists())


if __name__ == '__main__':
    unittest.main()
