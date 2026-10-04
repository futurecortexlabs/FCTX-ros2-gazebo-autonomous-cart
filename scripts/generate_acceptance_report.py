"""基本受入の全段階の合格を確認してから報告書を作る。仕上げの追加試験は集計しない。"""
from pathlib import Path
import json,datetime
ROOT=Path(__file__).resolve().parents[1]
def read(name):return json.loads((ROOT/'logs'/name).read_text())
expected={'test_startup_supervision.py','validate_all_localization.py','validate_startup_recovery.py','validate_endurance.py','validate_optimization.py','test_course_switch.py','validate_gazebo_shutdown.py'}
stages=read('acceptance_results.json');assert {x['test'] for x in stages}==expected and all(x.get('exit_code')==0 or (x['test']=='validate_gazebo_shutdown.py' and x.get('result')=='passed' and x.get('evidence')=='gazebo_shutdown_validation.json') for x in stages)
cases=read('all_localization_validation.json');assert len(cases)==7 and len({x['course'] for x in cases})==7
for case in cases:
 assert case['passed']
 for mode in ('slam','localization'):
  assert case[mode]['collision_count']==0 and case[mode]['endpoint_error_m']<.35
endurance=read('endurance_validation.json');assert endurance['passed'] and endurance['elapsed_seconds']>=600 and endurance['completed_cycles']>=2
regression=read('optimization_validation.json');assert len(regression)==15 and all(x['exit_code']==0 for x in regression)
assert read('startup_recovery_validation.json')['passed']
shutdown=read('gazebo_shutdown_validation.json');assert len(shutdown)==8 and all(x['passed'] for x in shutdown)
assert read('course_switch_validation.json')['all_passed']
now=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=9))).isoformat(timespec='seconds')
lines=['# FCTX-シミュレーション基本受入報告',f'作成: {now}','','基本受入の7段階をすべて通過。この集計は短距離・10分試験の範囲で、仕上げの全周・30分試験を含まない。仕上げの条件と結果は [ACCEPTANCE.md](../ACCEPTANCE.md) と [検証と適用範囲](../docs/VALIDATION.md) を参照。','','## 修正した問題','','- 4輪旋回の車輪滑りによるSLAM位置ずれ: IMUと車輪速度のEKF融合、周回壁の識別用目印を追加。','- 停止後の位置情報待ち: SLAMのTF更新と発進時の照合待ちを修正。走行中の照合断は新しい照合まで停止を保持。','- 大型倉庫の未観測通路: LiDARとSLAM/AMCLの最大距離を35mへ拡張。','- 大型地図の目的地入力: 20mの固定上限を地図に合わせて拡張。','- 起動・終了: 準備状態の監視、60秒タイムアウト、起動中だけ1回再試行、Gazebo一時停止の実適用を確認してから所有launchへSIGINTを送り、無応答時の段階的回収を追加。','','## 全7コースのSLAM・保存地図配送','','各コースで旋回観測、1.2m往復配送、地図保存、AMCL再起動、再配送、完了停止を確認。接触はすべて0。以下は往復終了時の推定位置と外部正解位置の差。','','| コース | SLAM誤差 | AMCL誤差 |','|---|---:|---:|']
for c in cases:lines.append(f"| {c['course']} | {100*c['slam']['endpoint_error_m']:.1f} cm | {100*c['localization']['endpoint_error_m']:.1f} cm |")
lines+=['','## 連続運転',f"倉庫のAMCL配送を{endurance['elapsed_seconds']/60:.1f}分間、{endurance['completed_cycles']}往復実行。接触0、全配送完了。制御状態の最大受信間隔は{endurance['max_status_age_seconds']:.3f}秒。"]
rss=endurance.get('gui_rss_kb',[])
if rss:lines.append(f'操作画面のRSS: 最初の配送後 {rss[0]/1024:.1f} MiB、最後 {rss[-1]/1024:.1f} MiB。')
lines+=['','## 安全・起動・回帰試験','','起動中の終了を注入し、1回の再試行後に手動停止で復帰。起動完了後の終了では再起動しない。既存の15本の検証スクリプトも全成功。AMCL停止時の安全停止、追加障害物の迂回、7コースの切替・自由回避を確認。','','## 範囲','','保存地図は実際に観測した範囲。棚の裏など未観測の場所は通行不可で、全域の完全な地図ではない。10分試験は長期・実機での信頼性保証ではない。再現した終了時クラッシュはWSLのlibd3d12core.soの早期アンロードが原因で、Gazeboプロセス内の保持で対策した。対策前6/6異常終了、対策後6/6正常終了に加え、7コースと表示ありAMCLの正常終了を確認。過去の単発の起動時getenv内クラッシュは同一原因と断定しておらず、起動監視と限定再試行も維持する。','','起動・復旧: [RUNBOOK.md](../RUNBOOK.md)。再検証: `python3 scripts/run_acceptance.py`。詳細ログ: `logs/acceptance.log`。']
report=ROOT/'logs/acceptance_report.md'
report.parent.mkdir(parents=True,exist_ok=True)
report.write_text('\n'.join(lines)+'\n',encoding='utf-8');print(report)
