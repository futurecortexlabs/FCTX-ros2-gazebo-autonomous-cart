"""起動環境・地図整合性・直近の状態を読み取り、診断情報をJSONへ保存する。"""
from pathlib import Path
import os,json,hashlib,subprocess,importlib.util,time,yaml
ROOT=Path(__file__).resolve().parents[1]
packages=['ros-jazzy-ros-gz-sim','ros-jazzy-ros-gz-bridge','ros-jazzy-slam-toolbox','ros-jazzy-nav2-amcl','ros-jazzy-nav2-map-server','ros-jazzy-nav2-lifecycle-manager','ros-jazzy-robot-localization']
result={'packages':{},'python':{},'maps':{},'display_configured':bool(os.environ.get('DISPLAY'))}
for package in packages:
 p=subprocess.run(['dpkg-query','-W','-f=${Version}',package],text=True,capture_output=True)
 result['packages'][package]=p.stdout if p.returncode==0 else None
for name in ['rclpy','numpy','scipy','yaml','PIL','PyQt5','psutil']:result['python'][name]=importlib.util.find_spec(name) is not None
for world in sorted((ROOT/'worlds').glob('*course.sdf')):
 folder=ROOT/'maps'/world.stem
 try:
  metadata=json.loads((folder/'map_metadata.json').read_text());config=yaml.safe_load((folder/'map.yaml').read_text())
  assert metadata['world_sha256']==hashlib.sha256(world.read_bytes()).hexdigest(),'コース変更後の古い地図'
  assert (folder/config['image']).is_file(),'地図画像がありません'
  result['maps'][world.name]='ready'
 except (OSError,ValueError,KeyError,AssertionError) as exc:result['maps'][world.name]=str(exc)
status=ROOT/'logs/loop_status.json'
if status.exists():
 result['status_age_seconds']=time.time()-status.stat().st_mtime
 try:result['last_status']=json.loads(status.read_text())
 except ValueError:result['last_status']='書き込み中または不正JSON'
result['dependencies_ready']=all(result['packages'].values()) and all(result['python'].values())
# クローン直後も、まだ存在しない実行ログ用フォルダへ診断を保存する。
(ROOT/'logs').mkdir(parents=True,exist_ok=True)
(ROOT/'logs/environment_check.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
print(json.dumps(result,ensure_ascii=False,indent=2))
raise SystemExit(0 if result['dependencies_ready'] else 1)
