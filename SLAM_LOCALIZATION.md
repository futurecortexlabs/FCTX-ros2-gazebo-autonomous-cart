# FCTX-SLAM・自己位置推定

## 使い方

起動: `bash run_courses.sh --course warehouse_course.sdf --pose-source localization --start`
全7コースの観測済み地図を同梱しています。地図は全周・主要通路を走行してLiDARで観測したものです。出発地点からつながる安全領域の観測率を測定しています。壁の内側の孤立領域や未知セルは通行できません。自己位置推定が始まったら、右の地図で目的地を選んで移動・配送できます。地図座標の原点はSLAM開始時の台車位置で、従来のworld座標とは異なります。

別コースの地図を作る場合:

1. 左の位置モードで「SLAM 地図作成」を選び、「選択したコースを開始」を押します。
2. 手動または自動回避でゆっくり走り、周囲を観測します。初めにその場旋回すると地図の観測が増えます。
3. 右側に実測した占有地図を表示します。薄い灰色は通行可能、濃い灰色は未観測、黒は障害物です。未知領域は経路計画では通行不可です。
4. 「地図を保存」を押します。maps/コース名/map.yaml と map.pgm、対応コースの情報を保存します。
5. 「保存地図で自己位置推定」を選び、同じコースを再開します。AMCLは出発地点付近（map座標0,0,0）を初期推定として開始します。
6. 初期推定がずれた場合は、地図の現在位置を選び、左の初期方向を設定して「選択位置を初期位置に」を押します。手動停止へ切り替えてから推定し直します。

地図を作っていないコース・内容が変わったコースでは、保存地図での起動を拒否します。配送ルートもworld/map座標を区別し、違う座標系のルートは読み込めません。全周・主要通路のmap座標ルートは `routes/*_coverage_delivery.json`、短い往復のサンプルは `routes/*_localized_delivery.json` です。地図を作り直した場合はルートの座標対応も確認してください。

## 構成

- SLAM: ROS 2 Jazzyのslam_toolbox。LiDARとIMU・車輪速度を融合したオドメトリで地図・map→wheel_odom補正を生成。
- 自己位置推定: nav2_amcl。保存した実測地図とLiDAR、IMU・車輪速度を融合したオドメトリを使用。
- TF: map → wheel_odom → loop_cart/base_link → loop_cart/lidar。正解位置のTFはSLAM/AMCLモードで配信しません。
- 制御位置: /loop/localized_odom。位置補正の古さ、走行中の照合更新停止、AMCLの大きい共分散を検出すると配信を止め、安全ゲートが停止します。
- 地図: /map のOccupancyGridを距離場へ変換。車体半径0.62mと余裕を膨張し、A*で経路計画します。SLAM/AMCLモードではSDF形状を経路に使用しません。
- 正解位置 /loop/ground_truth は外部精度検証・速度表示用に残っています。自動制御、安全ゲート、位置推定ノードは購読しません。

設定は config/slam.yaml、config/amcl.yaml、config/ekf.yaml。プログラムは scripts/occupancy_navigation.py、localization_bridge.py、save_slam_map.py。

## 範囲と注意

地図作成・保存・再起動後のAMCL・目的地移動・配送に対応。探索していない領域へは移動できません。対称形のコースでは同じような景色が続くため、正しい初期位置と十分な観測が必要です。全コースの全位置について精度を保証するものではありません。

地図上の箱追加操作はworld座標モードだけで使用できます。SLAM/AMCLでも実際のLiDAR反射による追加障害物の検知と再計画は有効ですが、UIから箱を置く座標変換はまだ提供していません。

## 参照

- slam_toolbox公式: https://docs.ros.org/en/jazzy/p/slam_toolbox/
- Nav2 AMCL公式: https://docs.nav2.org/configuration/packages/configuring-amcl.html

## 検証

占有地図・位置中継・制御の単体試験は `scripts/test_occupancy_navigation.py`、`scripts/test_localization_bridge.py`、`scripts/test_goal_controller.py` です。通常のシミュレーションを終了してから実行してください。

全7コースの共通SLAM設定、付属地図でのAMCL配送、位置更新断時の停止・復帰の結果と再実行方法は [検証資料](docs/VALIDATION.md) にまとめています。

## IMU・車輪速度の融合

`config/ekf.yaml`でrobot_localizationのEKFを設定。`/loop/wheel_odom`の前後・横速度と`/loop/imu`の相対姿勢・角速度から`wheel_odom → loop_cart/base_link`を配信する。生の車輪TFとの二重配信はしない。SLAM/AMCLはこのTFとLiDARを使用する。IMUの更新断は停止条件。追加パッケージは`ros-jazzy-robot-localization`。

全コースの受入試験と観測範囲は`ACCEPTANCE.md`を参照。保存地図はコースのハッシュで照合するため、コース変更後は地図の再作成が必要。

停止中はSLAMの地図補正TFを現在時刻で配信する。発進から最初の照合には1.5秒の猶予を設けるが、走行中の照合断を検出すると、新しい照合が届くまで停止を保持する。

LiDAR最大距離は35m。大型倉庫の外周壁が15mより遠いため、十分な反射を得る設定に拡張した。未知セルを安全判定から除外する処理は行っていない。

最終版の保存済み結果は [validation/20261003/](validation/20261003/)、条件と再実行方法は [検証と適用範囲](docs/VALIDATION.md) を参照してください。

## SLAM照合の方向設定

対称な周回壁では、LiDAR照合が別の向きでも一致することがあります。`config/slam.yaml` の `angle_variance_penalty: 0.0025`、`minimum_angle_penalty: 0.2` は、IMU/EKFの方向から大きく回転した候補の照合得点を下げる設定です。[slam_toolboxの照合実装](https://github.com/SteveMacenski/slam_toolbox/blob/ros2/lib/karto_sdk/src/Mapper.cpp#L637-L649) に対応します。

最終設定の全7コース・105検査地点での結果は [SLAM検証記録](validation/20261003/final_slam_regression_summary.json) を参照してください。正解位置は独立した外部検証で測定し、制御・SLAM・EKFへ入力しません。実機のIMUバイアスを含む条件は別の調整・評価が必要です。

## 閉路探索の設定

閉路認識は有効にし、`loop_search_space_dimension: 1.0`、`loop_search_maximum_distance: 0.75` を設定しています。[公式実装](https://github.com/SteveMacenski/slam_toolbox/blob/ros2/lib/karto_sdk/src/Mapper.cpp)では閉路照合時の角度ペナルティが無効になるため、通常照合の方向設定と閉路探索を分けて調整します。

1.0mは粗探索の範囲です。精探索とグラフ最適化もあるため、最終補正量が厳密に±0.5mへ制限される設定ではありません。付属地図は生成後にAMCLの全巡回を検証したものを採用しています。地図生成時の設定と共通最終設定の回帰試験は、[検証資料](docs/VALIDATION.md)で区別します。

## 低速時の照合更新

移動量の間引き閾値は `minimum_travel_distance: 0.02` m、`minimum_travel_heading: 0.02` radです。低速でも照合を更新できるようにし、1.5秒の停止監視条件は維持します。

0.05m/s・25秒の診断では走行中の更新待ち表示が0標本、最大位置受信間隔が0.46秒でした。1秒間隔の観測なので短い完全停止の有無までは断定しません。別の0.5m目標でも経路計画・到着判定、接触0、正常終了を確認しています。[低速条件の診断](validation/20261003/low_speed_slam_comparison.json)。全周観測の合格を測る診断ではなく、生成地図は復元しています。

## 終了時のSLAM配信停止

操作画面はGazebo一時停止の実反映を確認し、SLAMのlifecycle deactivateが完了してから所有launchへSIGINTを送ります。[公式deactivate実装](https://github.com/SteveMacenski/slam_toolbox/blob/2.8.5/src/slam_toolbox_common.cpp#L163-L172)は配信スレッドを停止・joinします。起動途中にSLAMを中止する場合は、位置・地図・状態の準備を最大20秒待ってから終了へ進みます。

サービスが使えない場合も要求を有限時間で回収し、所有launchの終了へ進みます。強制終了とnative異常を正常終了の合格には含めません。実終了の確認は [SLAM終了記録](validation/20261003/lifecycle_shutdown_verification.json)と [試験条件・版の区別](docs/VALIDATION.md) を参照してください。
