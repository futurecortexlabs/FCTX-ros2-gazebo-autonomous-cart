#!/usr/bin/env python3
# ファイルの役割: 開始方向を変えたSDFでGazeboを起動し、周回進捗・接触・距離をJSONから検証する。
"""Real Gazebo validation from opposite headings and directly facing a wall."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]


# 開始姿勢を変えた試験SDFを保存し、Gazeboの実走結果を監視する。成功・失敗にかかわらず子プロセスを終了する。
def run_case(name, yaw, expected_sign, laps, seconds, auto_speed=.45, source_world=None):
    tree=ET.parse(source_world or ROOT/'worlds/loop_course.sdf')
    tree.find(".//model[@name='loop_cart']/pose").text=f'4.5 0 0.18 0 0 {yaw}'
    world=ROOT/'logs'/f'avoidance_{name}.sdf'
    tree.write(world,encoding='utf-8',xml_declaration=True)
    report=ROOT/'logs'/f'avoidance_{name}.json'
    if report.exists():
        report.rename(report.with_name(report.stem+'_'+str(int(time.time()))+'.json'))
    with (ROOT/'logs'/f'avoidance_{name}.log').open('w') as log:
        proc=subprocess.Popen(['bash',str(ROOT/'run_loop.sh'),'gui:=false','controls:=false',f'auto_speed:={auto_speed}',f'world:={world}',f'laps:={laps}',f'report:={report}'],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        print('START',name,flush=True)
        deadline=time.monotonic()+180
        last_lap=-100
        try:
            while time.monotonic()<deadline:
                if proc.poll() is not None:
                    raise RuntimeError(f'{name}: launch exited; see {log.name}')
                if report.exists():
                    data=json.loads(report.read_text())
                    assert data.get('collision_count', data['wall_contact_count'])==0,(name,data)
                    if abs(data['laps']-last_lap)>.24:
                        print(name,'laps',data['laps'],'clearance',data['min_body_wall_clearance_lower_bound_m'],'state',data['safety_state'],flush=True)
                        last_lap=data['laps']
                    if data['completed'] or (not laps and data['sim_seconds']>=seconds):
                        break
                time.sleep(.5)
            else:
                raise TimeoutError(name)
        finally:
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGINT)
            try:
                proc.wait(timeout=22)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGKILL)
                proc.wait(timeout=8)
    data=json.loads(report.read_text())
    assert data.get('collision_count', data['wall_contact_count'])==0
    assert data['valid_scan_messages']>200
    assert data['min_body_wall_clearance_lower_bound_m']>.12,(name,data)
    if expected_sign:
        assert data['completed'] and data['laps']*expected_sign>=1.,(name,data)
    else:
        assert abs(data['laps'])>.25,(name,data)
    print('PASS',name,'laps',data['laps'],'clearance',data['min_body_wall_clearance_lower_bound_m'],flush=True)
    return dict(case=name,initial_yaw=yaw,**data)


if __name__=='__main__':
    results=[run_case('counterclockwise',math.pi/2,1,1,0),
             run_case('clockwise',-math.pi/2,-1,1,0),
             run_case('facing_wall',0,0,0,35)]
    (ROOT/'logs/avoidance_validation.json').write_text(json.dumps({'all_passed':True,'cases':results},indent=2)+'\n')
    print('ALL THREE GAZEBO SCENARIOS PASSED',flush=True)
