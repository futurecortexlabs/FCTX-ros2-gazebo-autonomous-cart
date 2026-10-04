"""既存の安全・操作・目的地テストを順番に実行し結果を保存する。"""
import subprocess,json,os
from pathlib import Path
root=Path(__file__).resolve().parents[1]
tests=['test_loop_core.py','test_obstacle_avoidance.py','test_manual_panel.py','test_goal_navigation.py','test_goal_controller.py','test_goal_panel.py','test_safety_gate.py','test_manual_mode.py','test_goal_course_switch.py','test_course_switch.py']
results=[]
for test in tests:
    print('RUN',test,flush=True)
    env=os.environ.copy();env['QT_QPA_PLATFORM']='offscreen';env['ROS_DOMAIN_ID']='42'
    p=subprocess.run(['python3',str(root/'scripts'/test)],cwd=root,env=env)
    results.append({'test':test,'exit_code':p.returncode})
    (root/'logs/audit_results.json').write_text(json.dumps(results,indent=2))
    if p.returncode:break
