# FCTX-起動と復旧

初回の依存導入は [セットアップ](docs/SETUP.md) を参照してください。以下はクローンしたリポジトリのルートから実行します。

## 通常の起動

```bash
bash run_courses.sh --course warehouse_course.sdf --pose-source localization --start
```

位置と地図の準備が完了すると「読み込み済み」と表示されます。開始時は手動・停止。目的地を地図で選び開始するか、「配送ミッション」から `routes/warehouse_course_localized_delivery.json` を読み込みます。

```bash
# 位置モードとコースを画面で選ぶ
bash run_courses.sh
# 既知のシミュレーション位置で大型倉庫を開始
bash run_courses.sh --course large_warehouse_course.sdf --pose-source simulation --start
# 倉庫のSLAM地図作成
bash run_courses.sh --course warehouse_course.sdf --pose-source slam --start
```

画面を閉じると、その画面から起動したGazeboも終了します。「コース終了」はGazeboを終了して操作画面を残します。次のコースは終了完了後に読み込みます。

単一コースを直接起動する場合は、次の入口も使用できます。

| コマンド | 動作 |
|---|---|
| `bash run_loop.sh` | 円形コースを自由回避で開始 |
| `bash run_manual.sh` | 円形コースを手動停止で開始 |
| `bash run_obstacles.sh` | 障害物コースを自由回避で開始 |
| `bash manual_control.sh` | 起動済みコースに手動操作パネルを追加 |

単一コースの操作パネルを閉じてもGazeboは残ります。シミュレーション全体を終了する場合はGazebo画面を閉じるか、起動ターミナルでCtrl+Cを押します。

## 地図を広げる

「SLAM 地図作成」で開始し、手動でゆっくり周囲を観測して「地図を保存」を押します。その後「保存地図で自己位置推定」で同じコースを再開します。未観測の灰色領域へは経路を作りません。[SLAM・自己位置推定](SLAM_LOCALIZATION.md)

## 起動しない場合

起動中の失敗は一度だけ自動再試行します。位置や地図が60秒で準備できなければ停止します。起動完了後の異常終了では自動再開しません。「コース終了」で終了を待ち、「選択したコースを開始」で再開してください。

二重起動の表示が出る場合は、既に動いているこのソフトの画面を閉じます。ロックファイルの削除だけで解除しないでください。ロックはプロセスが終了するとOSが解除します。

環境診断：

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/check_environment.py
```

| 記録 | 内容 |
|---|---|
| `logs/environment_check.json` | 依存・地図整合性・画面環境・状態ファイルの古さ |
| `logs/course_selector.log` | コースの起動と終了 |
| `logs/loop_status.json` | モード・速度・接触・配送・位置情報 |

状態ファイルが古い場合は以前の実行結果です。診断の `status_age_seconds` を確認してください。WSLでウィンドウが出ない場合はWSLgと `DISPLAY`、日本語表示はNoto Sans CJK JPの導入を確認します。

## 検証

画面からコースを終了してから、単体・初回診断は `python3 scripts/validate_release.py --checks units runners fresh-copy`、付属地図の全配送は `python3 scripts/validate_saved_routes.py` で確認します。基本受入の `run_acceptance.py` は地図を短い観測結果へ置き換えるので、既存地図を保持する場合は別コピーで実行してください。必要な環境・更新されるファイル・結果の読み方は [検証と適用範囲](docs/VALIDATION.md)、判定条件は [ACCEPTANCE.md](ACCEPTANCE.md) を参照してください。

## 終了待ちが長い場合

終了時はゼロ指令、一時停止要求、統計で停止確認、SLAMの場合はlifecycle deactivate、所有ROS launchへのSIGINTの順に進みます。終了待ち中は操作を無効にし、旧コースと終了要求を回収してから戻します。起動中の中止・準備直後・コース切替・GUI終了の9条件で正常終了と残存プロセスなしを確認しました。

一時停止のCLIが応答しない場合も待ち時間には上限があります。画面の終了待ちが続く場合は `logs/course_selector.log` を確認し、終了が確定するまで次のコースを開始しないでください。

## WSLの描画ライブラリ対策

`launch/loop_course.launch.py`は、WSLの `/usr/lib/wsl/lib/libd3d12core.so` が存在する場合に、描画ライブラリをGazeboのプロセス内で保持します。描画スレッドより先に解放されて終了時に落ちる問題への対策です。通常の起動で適用されます。
