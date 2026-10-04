#!/usr/bin/env python3
# ファイルの役割: 障害物コースを両開始方向と高速設定で試走し、1周完了と非接触を検証する。
"""Physical obstacle-course regression: both starting headings and high speed."""
import json
import math
from validate_avoidance import ROOT, run_case


if __name__ == '__main__':
    results = []
    for name, yaw, speed in [('obstacles_forward', math.pi/2, .45),
                             ('obstacles_reverse', -math.pi/2, .45),
                             ('obstacles_fast', math.pi/2, 1.5)]:
        result = run_case(name, yaw, 0, 1, 0, speed, ROOT/'worlds/obstacle_course.sdf')
        assert result['completed'], result
        assert result['obstacle_contact_count'] == 0, result
        assert result['min_lidar_distance_m'] > .78, result
        results.append(result)
    (ROOT/'logs/obstacle_validation.json').write_text(json.dumps(
        {'all_passed': True, 'cases': results}, indent=2)+'\n')
    print('ALL OBSTACLE SCENARIOS PASSED', flush=True)
