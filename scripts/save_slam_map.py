"""SLAM占有地図をPGM/YAMLで保存し、対応コース情報も記録する。"""
import argparse,hashlib,json,os,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def save(world):
    world=Path(world).resolve()
    if world.parent!=ROOT/'worlds' or not world.is_file():raise ValueError('コースが不正です')
    report=json.loads((ROOT/'logs/loop_status.json').read_text())
    if report.get('localization')!='slam' or report.get('course_world')!=str(world):raise ValueError('該当コースのSLAMモードで保存してください')
    folder=ROOT/'maps'/world.stem;folder.mkdir(parents=True,exist_ok=True)
    # 失敗時に以前の地図を壊さないよう、一時名へ出力してから置き換える。
    prefix=folder/'pending_map'
    env=os.environ.copy();env.setdefault('ROS_DOMAIN_ID','42')
    subprocess.run(['ros2','run','nav2_map_server','map_saver_cli','-f',str(prefix),'--ros-args','-p','save_map_timeout:=10.0','-p','map_subscribe_transient_local:=true'],env=env,check=True,timeout=20)
    import yaml
    data=yaml.safe_load(prefix.with_suffix('.yaml').read_text());image=Path(data['image'])
    if not image.is_absolute():image=folder/image
    final_image=folder/('map'+image.suffix);image.replace(final_image)
    data['image']=final_image.name
    (folder/'map.yaml').write_text(yaml.safe_dump(data))
    prefix.with_suffix('.yaml').unlink()
    (folder/'map_metadata.json').write_text(json.dumps(dict(world=world.name,world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(),frame='map',source='slam_toolbox',saved_at=time.time()),indent=2))
    return folder/'map.yaml'
if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('world');args=parser.parse_args()
    try:print('地図を保存しました: '+str(save(args.world)))
    except Exception as exc:print(str(exc));raise SystemExit(1)
