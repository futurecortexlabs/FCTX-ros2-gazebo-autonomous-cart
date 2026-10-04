# FCTX-目的地まで移動する

## 起動と操作

```bash
cd ~/FCTX-ros2-gazebo-autonomous-cart
bash run_courses.sh
```

1. 左上でコースと位置モードを選び、「選択したコースを開始」を押します。
2. 位置と地図の準備後、右側の地図をクリックするか、X・Y（m）を入力します。黄色が選択位置です。
3. 「この目的地へ移動」を押します。経路が受理されると自動モードに切り替わります。
4. 緑色の経路に沿って進み、制御用の現在位置と目的地との距離が0.18m以内になると停止します。

上が+Y、右が+Xです。青い台車から白い線が伸びている向きが前方です。壁・障害物の内部、近すぎる点、経路がつながっていない点は拒否します。SLAM/AMCLでは未観測の場所へ経路を作りません。

到着後と「目的地を取消して停止」の操作後は停止を維持します。「自動回避」を押すと目的地を解除し、LiDARによる自由回避へ移ります。手動への切替は進行中の目的地を取り消します。停止ロックは移動開始ボタンで解除されず、自動速度ゼロでも待機します。コース切替時は目的地と経路をリセットして手動停止で開始します。

## 位置モードと座標系

| 位置モード | 現在位置 | 経路計画の地図 | 目的地の座標系 |
| --- | --- | --- | --- |
| シミュレーション位置 | Gazeboの正解位置 | SDFに定義した静的形状 | `world` |
| SLAM 地図作成 | `slam_toolbox`による推定位置 | 観測中の占有地図 | `map` |
| 保存地図で自己位置推定 | 保存地図でのAMCL推定位置 | 保存した占有地図 | `map` |

SLAM/AMCLでは、IMUと車輪速度を融合したEKFオドメトリを使用します。Gazeboの正解位置とSDF形状は、これらのモードの位置推定・経路計画には使用しません。地図座標の原点はSLAM開始時の台車位置で、`world`の原点とは異なります。位置モードを変更したときは座標値をそのまま流用せず、現在の地図で目的地を選び直してください。

保存地図で起動する例:

```bash
bash run_courses.sh --course warehouse_course.sdf --pose-source localization --start
```

位置推定・地図保存・初期位置の再設定は [SLAM_LOCALIZATION.md](SLAM_LOCALIZATION.md) を参照してください。

## 経路計画と走行制御

経路計画・追従は自作のPython実装です。Nav2のAMCL・地図サーバーを位置推定に使用しますが、Nav2のナビゲーション一式を使用する構成ではありません。

車体とタイヤを覆う半径0.62mに余裕を加え、A*で通行可能な経路を探索します。シミュレーション位置モードのグリッドは0.10m刻みです。SLAM/AMCLでは占有地図の解像度を使用し、未知セルと地図外を通行不可にします。斜め移動で壁の角を抜けることを禁止し、経路を短くまとめる際も線分の余裕を検査します。

経路追従では、曲がり角の方向へ旋回してから前進します。目的地走行の並進速度は設定した自動上限以下、かつ最大0.55m/sです。0.18mの到着判定は制御用位置に対する値であり、SLAM/AMCLの実際の到着精度には位置推定誤差も影響します。目的地での最終姿勢指定には対応していません。

LiDARで検知した追加障害物が経路を塞ぐ場合は停止し、迂回経路を再計画します。通れる経路がなければ停止を保持して再確認します。詳細は [DYNAMIC_OBSTACLES.md](DYNAMIC_OBSTACLES.md) を参照してください。並進・旋回の進捗が12秒間ない場合も停止を保持します。

最終速度指令は独立した`SafetyGate`を通ります。手動・自由回避・目的地走行で、LiDAR接近、指令やセンサーの更新断、接触などによる停止条件を共有します。位置推定が不確か・古い場合にも走行を止めます。この構成はシミュレーション用で、実機での安全保証を示すものではありません。

## 主なファイルとROSインターフェース

| ファイル | 役割 |
| --- | --- |
| `scripts/goal_navigation.py` | A*計画、経路追従、到着・取消・進行不能時の停止保持 |
| `scripts/occupancy_navigation.py` | 占有地図から通行余裕を計算。未知・地図外は通行不可 |
| `scripts/goal_panel.py` | 地図描画、目的地選択、ROS要求と応答の確認 |
| `scripts/loop_controller.py` | 目的地受付、自由回避・配送の選択、再計画、状態配信 |
| `scripts/world_clearance.py` | シミュレーション位置モードで使用するSDF形状の読込 |

| ROS名 | 型 | 内容 |
| --- | --- | --- |
| `/loop/goal` | `geometry_msgs/msg/PoseStamped` | `position.x/y`を目的地として使用。`header.frame_id`は現在のモードに合わせて`world`または`map` |
| `/loop/cancel_goal` | `std_srvs/srv/Trigger` | 目的地を取消し、停止を保持 |
| `/loop/free_drive` | `std_srvs/srv/Trigger` | 目的地を解除して自由回避へ変更 |
| `/loop/navigation_status` | `std_msgs/msg/String` | 状態・目的地・経路・位置・座標系・配送進捗などをJSONで配信 |
| `/loop/planned_path` | `nav_msgs/msg/Path` | 現在の座標系での計画経路 |

目的地をROSから送るだけでは手動モードを解除しません。操作画面は要求の受理を確認した後に、`/loop/set_manual`へ`false`を送ります。自由回避も、安全ゲートのモードが`auto`であり、停止条件が解除されていることが走行の前提です。

## 検証

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/test_goal_navigation.py
python3 scripts/test_goal_controller.py
python3 scripts/test_goal_panel.py
```

GazeboでのSLAM/AMCL検証は、通常のシミュレーションを画面から終了した後に実行します。

```bash
python3 scripts/validate_saved_routes.py
```

付属地図と巡回ルートを使用した全7コース・計98地点の配送、接触0、完了停止を確認しています。[保存済み結果](validation/20261003/saved_routes_summary.json)の`endpoint_error_m`は帰還時の推定位置と外部正解位置との差で、目的地への到着誤差とは別の指標です。試験条件と適用範囲は [検証と適用範囲](docs/VALIDATION.md)、判定条件は [ACCEPTANCE.md](ACCEPTANCE.md) を参照してください。
