# FCTX-コース選択と位置モード

## 起動と切替

クローンしたフォルダで起動します。動作環境の準備は [README.md](README.md) を参照してください。

```bash
cd ~/FCTX-ros2-gazebo-autonomous-cart
bash run_courses.sh
```

1. 左上の一覧でコースと位置モードを選びます。
2. 「選択したコースを開始」を押します。既存のコースがある場合は、終了を待ってから次のコースを起動します。
3. 新しいコースは手動・停止状態で開始します。キー操作、自由回避、目的地移動、配送ミッションを選んで操作します。

切替時は台車の初期位置へ戻り、目的地・経路・配送地点一覧・走行記録をリセットします。配送ルートは切替前に保存してください。「コース終了」またはコース選択画面を閉じると、この画面から起動したシミュレーションを終了します。別の起動スクリプトでシミュレーションが動いている場合は先に終了してください。

コースを指定して開始する例:

```bash
bash run_courses.sh --course warehouse_course.sdf --pose-source localization --start
```

## 7種類のコース

寸法は壁の配置を基準とする概略です。

| コース | SDFファイル | 構成 |
| --- | --- | --- |
| 円形 | `loop_course.sdf` | 内周半径3m・外周半径6mの壁を持つ基本コース |
| 障害物 | `obstacle_course.sdf` | 円形コースに箱2個とポール1個を追加 |
| 楕円 | `oval_course.sdf` | 外周15×12m・内周9×6mの周回通路 |
| 倉庫 | `warehouse_course.sdf` | 外壁14×10mと棚4個 |
| 大型楕円 | `large_oval_course.sdf` | 外周24×16m・内周16×8mの広い周回通路 |
| 大型倉庫 | `large_warehouse_course.sdf` | 外壁26×20mと3列×3段の棚9個 |
| スラローム障害物 | `slalom_course.sdf` | 外壁24×16mと交互に配置した壁3枚。各壁の先端に6mの開口 |

円形・楕円のコースには、SLAMが対称な壁を識別しやすくするため、形状と間隔の異なる紫色の目印を追加しています。

「自動回避」はLiDARを基準に進行方向を選びます。時計回り・反時計回りは固定しておらず、目的地へ必ず到達するモードではありません。目的地を指定する場合は [GOAL_NAVIGATION.md](GOAL_NAVIGATION.md)、複数地点を巡回する場合は [MISSION_GUIDE.md](MISSION_GUIDE.md) を参照してください。

## 3種類の位置モード

| 画面の選択肢 | CLI引数 | 制御用位置と経路計画 |
| --- | --- | --- |
| シミュレーション位置 | `--pose-source simulation` | Gazeboの正解位置とSDFの既知形状を使用。座標系は`world` |
| SLAM 地図作成 | `--pose-source slam` | LiDARとIMU・車輪速度を用いたSLAM位置、観測した占有地図を使用。座標系は`map` |
| 保存地図で自己位置推定 | `--pose-source localization` | 保存地図とLiDAR、IMU・車輪速度からAMCLで位置を推定。座標系は`map` |

SLAM/AMCLでは、経路計画にSDF形状やGazeboの正解位置を使用しません。未観測の領域は通行不可です。保存地図は7コース分を同梱していますが、観測した範囲の地図であり、各コースの全域を保証するものではありません。保存地図のないコースや、地図保存後にSDFが変わったコースではAMCL起動を拒否します。

`world`と`map`の座標は一致しません。位置モードに合った目的地・配送ルートを使用してください。地図作成・保存・初期位置の指定は [SLAM_LOCALIZATION.md](SLAM_LOCALIZATION.md) を参照してください。

## コース生成と検証

生成スクリプトは `scripts/build_loop_world.py`、`scripts/build_obstacle_course.py`、`scripts/build_extra_courses.py`、`scripts/build_large_courses.py` です。コースの変更や再生成後は、保存地図との整合性を確認し、必要に応じてSLAM地図を作り直します。

全7コースの切替・自由回避を確認した結果は [コース切替の検証記録](validation/20261003/final_course_switch_verification.json) に保存しています。SLAM全巡回、保存地図でのAMCL配送、試験条件と適用範囲は [検証と適用範囲](docs/VALIDATION.md) を参照してください。

再検証は、画面から通常のシミュレーションを終了した後に実行します。

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/test_course_switch.py
```

新しい結果は `logs/course_switch_validation.json` に保存します。周回数`laps`は原点周りの正味角度による指標で、倉庫の到着判定や走行距離には使用しません。
