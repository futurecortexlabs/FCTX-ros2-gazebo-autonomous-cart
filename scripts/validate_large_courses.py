"""追加コースで実走し、接触なしの到着と停止を既存の共通検証で確認する。"""
import json
from validate_goal_navigation import run_case, ROOT

if __name__ == '__main__':
    results=[]
    for world,goal in [('large_oval_course.sdf',(0.,6.)),('large_warehouse_course.sdf',(-3.,-2.)),('slalom_course.sdf',(-3.,4.))]:
        results.append(run_case(world,goal))
        (ROOT/'logs/large_courses_validation.json').write_text(json.dumps({'cases':results,'all_passed':len(results)==3},indent=2))
