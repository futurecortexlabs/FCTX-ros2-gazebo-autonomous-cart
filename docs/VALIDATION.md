# FCTX-検証結果と適用範囲

実走行は2026-10-03に Windows + WSL2 / Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic で検証しました。2026-10-04には個人情報の匿名化後の版で、単体試験と初回診断を再確認しました。試験ごとにROS domainとGazebo partitionを分け、所有するプロセスだけを起動・終了しています。[結果JSONの案内](../validation/20261003/README.md)

## 2026-10-05のMIT・GUI移行確認

現在のGUIはPySide2 5.15.13／Qt 5.15.13です。18入口の単体試験、実行器6条件、ログやローカルフォントを含まない別コピーでの初回診断・全7地図の整合性が成功しました。追加のQt2件では、奇数幅の地図画像の上下方向・一時バッファ破棄後の画素保持と、実QProcessの出力読込・終了通知・破棄・再実行を確認しています。[単体・初回診断の記録](../validation/20261005/mit_release_validation.json)

隔離したROS domain／Gazebo partitionと試験コピーで、倉庫AMCLの地図表示、AMCL停止注入時の安全停止と復帰、ルート保存・読込、2地点配送、接触0、完了後の実停止を確認しました。終了時はGazebo一時停止、所有launchの終了コード0、SIGINTのみ、所有プロセス残存0でした。[GUI統合・正常終了の記録](../validation/20261005/mit_gui_integration.json)

今回のGUI統合を再実行する場合は別コピーで `python3 scripts/check_environment.py` を実行してログ出力先を準備してから、`python3 scripts/validate_localized_ui.py` を実行します。この検証は試験コピー内の `routes/warehouse_slam_delivery.json` を書き換えるため、通常の作業フォルダでは実行しないでください。

以下の2026-10-03・10-04のGUIを含む結果はPyQt5を使用した当時の記録です。PySide2への移行はGUI依存と試験処理の変更で、走行制御・配布地図・SLAM設定は維持しました。全7コースのSLAM/AMCL巡回・30分連続配送・表示ウィンドウ付き描画は今回再実行していません。過去のソースSHAと今回の記録を区別して扱ってください。

## 配布地図とAMCL配送

付属する観測地図と全巡回ルートを使い、全7コースでAMCLを再起動しました。計98地点への配送、接触0、配送完了後の実停止を確認しました。

| コース | 安全領域の観測率 | 地図上の通行可能率 | 配送地点数 | AMCL帰還時の推定誤差 |
|---|---:|---:|---:|---:|
| 円形 | 100.0% | 88.1% | 16 | 3.33 cm |
| 障害物 | 100.0% | 86.7% | 16 | 0.82 cm |
| 楕円 | 100.0% | 87.7% | 16 | 4.52 cm |
| 倉庫 | 100.0% | 95.0% | 12 | 2.56 cm |
| 大型楕円 | 100.0% | 93.7% | 16 | 2.65 cm |
| 大型倉庫 | 100.0% | 96.0% | 8 | 0.63 cm |
| スラローム | 100.0% | 97.4% | 14 | 1.83 cm |

根拠：[配布地図の計測](../validation/20261003/shipped_map_coverage.json)、[全7コース配送・使用地図とルートのSHA](../validation/20261003/saved_routes_summary.json)。地点数は待機地点・帰還地点を含みます。推定誤差は帰還時の推定位置とGazebo正解位置との差で、目的地までの残距離や全経路の最大誤差とは異なります。

観測率は0.1m間隔の参照点で測定します。SDF外壁内から車体半径0.62mと余裕0.28mを考慮し、出発地点から連結した安全領域を分母にします。周回コースの内壁内など孤立領域は含まず、孤立領域を含む値はJSONの `all_enclosed_safe` に区別して記録します。SDFは外部評価の基準に使い、SLAM/AMCL制御用の地図はLiDAR観測から作ります。

観測率は参照点が既知の自由セルかを表します。通行可能率は障害物・未知領域を膨張した後の別指標です。観測率100%でも全参照点への走行を許可するものではありません。未知セルは通行不可とし、配送区間は製品と同じA*で確認します。

付属地図の生成時の設定・SHAは[地図の来歴確認](../validation/20261003/map_generation_provenance.json)に記録しています。共通最終設定のSLAM回帰で再生成した地図は試験コピーに保存し、AMCL配送済みの付属地図・ルートは置き換えていません。

## 共通最終設定の全7コースSLAM

最終のGUI・制御・SLAM設定で計105検査地点を巡回しました。全7コースで帰還、接触0、出発地点から連結した安全領域の観測率100%を確認しています。[統合結果・ソースSHA・終了確認](../validation/20261003/final_slam_regression_summary.json)

| コース | 検査地点 | 最大検査地点誤差 | 帰還時推定誤差 | 正常終了時間 |
|---|---:|---:|---:|---:|
| 円形 | 17 | 3.02 cm | 1.08 cm | 2.88 s |
| 障害物 | 17 | 2.46 cm | 0.96 cm | 4.99 s |
| 楕円 | 17 | 5.07 cm | 1.26 cm | 2.99 s |
| 倉庫 | 13 | 1.63 cm | 0.67 cm | 4.11 s |
| 大型楕円 | 17 | 8.49 cm | 1.11 cm | 4.45 s |
| 大型倉庫 | 9 | 12.67 cm | 1.95 cm | 3.60 s |
| スラローム | 15 | 2.04 cm | 0.35 cm | 3.20 s |

誤差は外部正解位置とSLAM推定位置の差です。検査地点での最大値であり、連続軌跡の最大値ではありません。正解位置は外部検証とGUIの速度表示で使い、制御・安全ゲート・EKF・位置中継の4ノードは非購読をROSグラフでも確認しました。

全7の所有launchは終了コード0、SLAM deactivate後のSIGINTでnative 56子プロセスが正常終了し、所有プロセス群と試験コピーの残存は0でした。円形・大型倉庫では検証器と観測器自身のOS終了コードを取得できず `null` と記録し、取得済み5コースの終了コード0と区別しています。

実走行の検証器と2026-10-03配布版には、終了判定を補強した3検証器と負例単体ファイルの4か所の版差があります。走行制御・GUI・位置中継・EKF・SLAM設定・worldは一致しました。実走行の終了はnativeログと全所有プロセス群の独立確認、新しい判定は単体試験で確認しています。停止後の旋回初動には1.5秒を超える位置受信間隔の標本があり、次の更新で復帰しました。障害物では安全監視の更新待ちを1標本観測し、次標本で復帰しています。1秒程度の観測では全ての短時間の事象を捕捉できず、生の指令時刻を採取していない標本の原因は断定していません。

低速0.05m/sではSLAMの移動量・旋回量の更新閾値を0.02m・0.02radとし、位置更新の停止監視1.5秒を維持しました。同じ25秒の比較で更新待ち4標本→0、最大位置受信間隔1.69秒→0.46秒でした。これは更新間隔の比較で、全コースの走行試験や完全停止の測定ではありません。0.5m目標の補足試験は到着・接触0・正常終了、生成地図は復元しています。[低速比較と測定範囲](../validation/20261003/low_speed_slam_comparison.json)

## 連続配送と安全停止

倉庫の12地点巡回を1851.0秒（30分51秒）・11巡回、計132地点実行しました。接触0、全配送完了、各巡回後の実停止を確認しました。帰還時の推定誤差は1.28〜2.34cm、採取した最大推定誤差は13.34cm、制御状態の最大受信間隔は0.333秒でした。GUIのRSSは最初の巡回後126472KB、最後126540KBです。[連続配送の結果](../validation/20261003/endurance_validation.json)、[使用ソース・配布地図の照合](../validation/20261003/endurance_source_verification.json)

追加障害物の配送試験はsimulationモードで再計画1回、接触0、到着・停止を確認しました。AMCLの停止注入はlocalizationモードで、位置更新停止時の停止と、再開後の2地点配送・停止を確認しました。[動的障害物](../validation/20261003/dynamic_route_validation.json)、[AMCL停止注入](../validation/20261003/localized_ui_validation.json)

## 終了・切替・描画

停止手順はゼロ指令→Gazebo一時停止→統計での実停止確認→SLAMの場合はlifecycle deactivate→所有launchへのSIGINTです。起動中のSLAM中止は準備を最大20秒待ってからdeactivateへ進みます。無応答時の回収は有限時間・所有プロセスに限定し、強制終了は正常終了の合格に含めません。

AMCL/simulationの9条件とSLAMの7条件で正常終了・所有プロセス残存0を確認しました。最終GUIの入力抑止追加後には4条件を再確認し、終了待ち中の入力遮断と終了後の復帰、所有launchの終了コード0、強制終了なしを確認しました。先の16条件と追加4条件は各試験時のソースSHAを分けて記録しています。[9条件の結果](../validation/20261003/shutdown_timing_validation.json)、[終了の独立確認](../validation/20261003/final_shutdown_verification.json)、[SLAM7条件](../validation/20261003/lifecycle_shutdown_verification.json)、[最終GUI4条件](../validation/20261003/shutdown_input_targeted_verification.json)

全7コースの自由回避・切替も接触0、各ケース1m超の移動、正常終了でした。起動中の異常終了は1回だけ再試行して手動停止へ復帰し、起動完了後は自動再開しません。同じWSLマシンで描画をllvmpipeへ変えた倉庫AMCL2地点往復は接触0・推定誤差2.48cm・正常終了でした。[全7コース切替](../validation/20261003/final_course_switch_verification.json)、[起動復旧](../validation/20261003/startup_recovery_validation.json)、[描画確認](../validation/20261003/software_renderer_final_verification.json)

検証器は正常終了とROS後処理の完了前に合格を保存しません。中断・後処理失敗・停止待ち中に増えた所有子の9条件を単体試験で確認しました。30分配送・全AMCL配送は当時の検証器版の結果を保持し、判定補強は短距離の実配送と終了、最終の中断処理は負例単体試験で確認しています。[終了判定の単体試験](../validation/20261003/final_validator_cleanup_units.json)、[正常終了の実統合確認](../validation/20261003/validator_shutdown_integration.json)

## 単体試験・初回診断・記録の版

17入口の単体試験と、実行器の成功・終了コード7・タイムアウト124を含む6条件が成功しました。ログ・Git・開発バックアップ・ローカルTTCフォントを含まないコピーで、初回ログ生成、依存、全7地図の整合性を確認しています。2026-10-03の最終実行ソース128ファイルと、2026-10-04の公開用整理後の版はそれぞれ記録します。[実走行時の静的・初回診断](../validation/20261003/final_release_validation.json)、[公開用整理後の確認](../validation/20261004/release_static_validation.json)

JSON中のソースSHAは各試験時の版を示します。公開用の匿名化や資料整理後に再測定した結果として扱いません。個人ディレクトリは `<workspace>` / `<home>` へ置き換え、実行時のプロセス番号・試験コピー名・非公開ログの保存先は公開用に除去しています。プロセスの終了・残存は件数として保持しています。元記録のSHAは匿名化前の記録を指し、公開用JSONの現在のファイル全体のSHAとは異なります。公開用の整理では文書と報告書の出力先を変更し、未使用の旧LiDAR学習サンプルと一度限りの試験再開スクリプトを削除しました。走行制御・配布地図・SLAM設定は維持しています。過去のソース一覧には、試験時に存在した削除済みファイルも含まれます。2026-10-04の保存記録は公開用匿名化後の確認であり、今回の資料削除後に17入口を再実行した結果ではありません。

## 再実行

[セットアップ](SETUP.md)後、通常運転を終了してからリポジトリのルートで実行します。

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/validate_release.py --checks units runners fresh-copy
python3 tests/test_map_coverage.py
python3 scripts/validate_map_coverage.py
python3 scripts/validate_saved_routes.py
python3 scripts/validate_endurance.py 30 --route routes/warehouse_course_coverage_delivery.json
python3 scripts/validate_shutdown_timing.py --include-gui
```

地図の計測だけでは地図を変更しません。SLAMを再作成する場合は別コピーで `python3 scripts/validate_map_coverage.py --explore --write-routes --mapping-speed 0.30` を実行します。地図・ルートを更新するため必要なデータを退避してください。基本受入の `run_acceptance.py` は短い観測地図に置き換える試験で、配布地図をそのまま保持するコマンドではありません。

既定domainは42、partitionは `ros2_gazebo_loop` です。外から指定した `ROS_DOMAIN_ID`・`GZ_PARTITION` は起動シェルと試験・障害物操作で維持します。同時実行する場合は両方を分離してください。ログは `logs/` に保存します。

## 適用範囲

- ROS 2/Gazeboシミュレーションの結果です。実機の滑り、センサー雑音・バイアス、通信、制動性能は別途評価が必要です。
- 観測率は指定した参照領域と格子の値で、孤立領域、未知セル、全域の位置精度保証を含みません。
- 自由回避は局所判断です。袋小路からの脱出やすべての目的地への到達は保証しません。
- 連続配送は30分51秒の有限試験です。初回診断と別描画経路は依存導入済みの同じWSL環境で確認し、別マシン・通常Linuxへの新規導入は未検証です。

受入条件は [ACCEPTANCE.md](../ACCEPTANCE.md) に記載しています。
