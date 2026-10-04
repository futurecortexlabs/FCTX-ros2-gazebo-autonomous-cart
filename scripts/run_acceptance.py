"""完成条件を再現する順次試験。失敗した段階で止まり、結果をJSONに保存する。"""
import subprocess,os,time,json,sys,shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
(ROOT/'logs').mkdir(parents=True,exist_ok=True)
if len(sys.argv)>1:
 path=Path('/proc')/sys.argv[1]/'cmdline'
 while path.exists():
  try:
   if b'validate_all_localization.py' not in path.read_bytes():break
  except OSError:break
  time.sleep(1)
 old=ROOT/'logs/all_localization_validation.json'
 if old.exists():shutil.copy2(old,old.with_name('all_localization_initial.json'))
tests=[('test_startup_supervision.py',[],30),('validate_all_localization.py',[],1500),('validate_startup_recovery.py',[],240),('validate_endurance.py',['10'],1000),('validate_optimization.py',[],900),('test_course_switch.py',[],400),('validate_gazebo_shutdown.py',[],600)]
results=[]
for test,args,timeout in tests:
 print('RUN',test,*args,flush=True)
 (ROOT/'logs/acceptance_progress.json').write_text(json.dumps(dict(stage=test,completed=results)))
 try:code=subprocess.run(['python3',str(ROOT/'scripts'/test),*args],cwd=ROOT,timeout=timeout).returncode
 except subprocess.TimeoutExpired:code=124
 results.append(dict(test=test,args=args,exit_code=code))
 (ROOT/'logs/acceptance_results.json').write_text(json.dumps(results,indent=2))
 if code:raise SystemExit(code)
print('ALL ACCEPTANCE STAGES PASSED',flush=True)
