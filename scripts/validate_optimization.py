"""最適化の回帰検証を順番に実行し、結果を記録する。"""
import os,subprocess,json
from pathlib import Path
root=Path(__file__).resolve().parents[1]
(root/'logs').mkdir(parents=True,exist_ok=True)
tests=['test_optimization.py','test_loop_core.py','test_obstacle_avoidance.py','test_goal_navigation.py','test_dynamic_obstacles.py','test_occupancy_navigation.py','test_localization_bridge.py','test_mission_control.py','test_goal_controller.py','test_manual_panel.py','test_goal_panel.py','test_safety_gate.py','test_manual_mode.py','validate_dynamic_route.py','validate_localized_ui.py']
results=[]
for test in tests:
 print('RUN',test,flush=True)
 env=os.environ.copy();env['QT_QPA_PLATFORM']='offscreen'
 try:code=subprocess.run(['python3',str(root/'scripts'/test)],cwd=root,env=env,timeout=300).returncode
 except subprocess.TimeoutExpired:code=124
 results.append(dict(test=test,exit_code=code))
 (root/'logs/optimization_validation.json').write_text(json.dumps(results,indent=2))
 if code:raise SystemExit(code)
