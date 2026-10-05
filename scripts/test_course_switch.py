# ファイルの役割: 実際のQt・ROS・Gazeboを使い、全コースの切替、停止状態での開始、自動走行と終了を確認する。
"""Exercise the actual Qt selector, ROS and Gazebo across all available courses."""
import os
os.environ['QT_QPA_PLATFORM'] = 'offscreen'
import json
import math
import time
import rclpy
from PySide2 import QtWidgets
from course_selector import CoursePanel, PanelNode, COURSES, ROOT, configure_japanese_font

app = QtWidgets.QApplication([])
configure_japanese_font(app)
rclpy.init()
node = PanelNode()
panel = CoursePanel(node, simulation_gui=False)
panel.show()
results = []


# Qtイベントを処理しながら条件成立を待つ。期限を超えたら表示状態を添えて失敗にする。
def until(predicate, seconds):
    deadline = time.monotonic()+seconds
    while time.monotonic() < deadline:
        app.processEvents()
        if predicate():
            return
        time.sleep(.02)
    raise TimeoutError(panel.course_status.text())


# 現在の走行JSONを読み、未作成・読出し途中の場合は空の辞書を返して次回の確認へ進む。
def report():
    try:
        return json.loads((ROOT/'logs/loop_status.json').read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


try:
    for index in range(len(COURSES)):
        panel.combo.setCurrentIndex(index)
        panel.switch_course()
        filename = COURSES[index][1]
        until(lambda: panel.active_course == index and panel.stopping_since is None
              and node.mode == 'manual' and time.monotonic()-node.state_time < .5
              and report().get('course_world', '').endswith(filename)
              and report().get('sim_seconds', 0) > 1, 45)
        assert abs(node.actual_speed) < .02, 'new course must start stationary'
        initial = report()['position']
        panel.set_mode(False)
        until(lambda: node.mode == 'auto' and abs(node.actual_speed) > .1, 10)
        began = time.monotonic()
        until(lambda: time.monotonic()-began > (22 if index >= 2 else 8), 30)
        data = report()
        assert data['collision_count'] == 0, data
        assert math.dist(initial[:2], data['position'][:2]) > 1., data
        assert data['min_body_environment_clearance_lower_bound_m'] > .12, data
        results.append(data)
        print('PASS switch/start/drive', filename, 'contacts', data['collision_count'], flush=True)
    panel.grab().save(str(ROOT/'logs/course_selector.png'))
finally:
    panel.close()
    until(lambda: panel.process is None, 35)
    app.processEvents()
    node.destroy_node()
    rclpy.shutdown()
(ROOT/'logs/course_switch_validation.json').write_text(json.dumps(
    {'all_passed': True, 'cases': results}, indent=2)+'\n')
print(f'ALL {len(COURSES)} COURSE SWITCHES PASSED', flush=True)
