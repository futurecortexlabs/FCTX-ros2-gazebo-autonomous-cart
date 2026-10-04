# FCTX-シミュレーション版の検証条件

対象はROS 2/Gazeboのシミュレーションです。以下は指定した試験条件での合格基準であり、無期限・全環境での無故障保証や実機の安全保証ではありません。測定結果・使用したソースの版・適用範囲は [検証資料](docs/VALIDATION.md)、個別結果は [保存記録](validation/20261003/) を参照してください。

## 判定条件

1. 全7コースをSLAMで全周・主要通路を巡回し、帰還と接触0を確認する。共通最終設定でも全7コースを検証する。
2. 出発地点からつながる参照安全領域の観測率90%以上。未知セルは通行不可とし、配送全区間でA*経路を生成できる。観測率と地図上の通行可能率は別の指標として測定する。
3. 付属地図と保存済み巡回ルートでAMCLを再起動し、全地点の配送・接触0・完了停止・帰還時の推定位置と外部正解位置との差0.35m未満を確認する。正解位置は外部検証にのみ使用し、SLAM/AMCLの制御・安全・位置推定へ入力しない。
4. 倉庫の12地点巡回を30分以上反復し、接触0、制御状態の受信間隔2秒未満、状態ファイルの更新間隔5秒未満、配送完了後の停止を確認する。
5. 起動中の中止、準備完了直後の終了、コース切替中の終了、GUI表示ありを含む9条件で正常終了し、所有プロセスが残らない。強制終了は合格に含めない。SLAMの終了条件と最終GUIの追加確認は、試験時のソースの版を区別する。
6. 追加障害物への再計画、AMCL停止注入時の安全停止・復帰、全7コースの自由回避・切替、単体試験、実行器の成功・失敗・タイムアウトを検証する。
7. 起動中の異常終了では1回だけ再試行し、手動停止で復帰する。起動完了後の異常終了では自動再開しない。位置・地図が60秒で準備できない場合は停止処理へ進む。
8. ログ・バックアップ・ローカルフォントを含まないコピーで初回診断が成功し、全7地図と依存を認識する。同じマシンのllvmpipe描画でも起動・配送・正常終了する。これは別マシンへの導入検証とは区別する。

位置誤差は帰還時の推定位置とGazebo正解位置の差であり、目的地までの残距離や連続軌跡全体の最大誤差とは異なります。観測率の分母は、車体半径と余裕を考慮した、出発地点からつながる参照領域です。内壁に囲まれた孤立領域は含みません。

## 再実行

[セットアップ](docs/SETUP.md)後、通常運転を操作画面から終了して、リポジトリのルートで実行します。

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/validate_release.py --checks units runners fresh-copy
python3 tests/test_map_coverage.py
# 付属地図の計測のみ。地図を変更しません。
python3 scripts/validate_map_coverage.py
# 付属地図・巡回ルートで全7コースのAMCL配送を検証します。
python3 scripts/validate_saved_routes.py
python3 scripts/validate_endurance.py 30 --route routes/warehouse_course_coverage_delivery.json
python3 scripts/validate_shutdown_timing.py --include-gui
```

SLAM地図の再作成は、別コピーで次を実行します。

```bash
python3 scripts/validate_map_coverage.py --explore --write-routes --mapping-speed 0.30
```

地図と巡回ルートを更新するため、保存したいデータは先に退避してください。検証中に不合格になったコースは、旧地図・ルートを復元します。

## 基本短距離試験

`python3 scripts/run_acceptance.py` は、短いSLAM観測・1.2m往復・10分配送を含む基本試験です。地図を短い観測結果へ置き換えるため、付属地図を保持する場合は別コピーで実行してください。上記の全周・30分試験を代替するものではありません。

結果は `logs/acceptance_results.json` などに保存します。`python3 scripts/generate_acceptance_report.py` は基本試験の全段階の合格を確認し、報告を `logs/acceptance_report.md` に生成します。
