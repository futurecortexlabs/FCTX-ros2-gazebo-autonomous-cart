"""既存の大型楕円で静的壁の誤検知と到着を確認する。"""
import json
from validate_goal_navigation import run_case,ROOT
result=run_case('large_oval_course.sdf',(0.,6.))
assert result['dynamic_obstacle_count']==0,result
assert result['replan_count']==0,result
(ROOT/'logs/dynamic_static_regression.json').write_text(json.dumps(result,indent=2))
print('PASS static walls are not dynamic obstacles',flush=True)
