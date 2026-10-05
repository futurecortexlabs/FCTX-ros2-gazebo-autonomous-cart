# FCTX-セットアップ

## 動作確認した環境

Windows上のWSL2、Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic、Python 3.12で検証しています。現在のGUIはQt5対応のPySide2を使用します。描画にはWSLgとD3D12を使用しました。通常のUbuntuデスクトップでも環境に応じた描画設定を使えますが、保存済みの受入試験はWSL環境の結果です。

このリポジトリはPythonスクリプトとROS launchを直接実行します。`colcon build`は必要ありません。

## ROS 2と依存パッケージ

Ubuntu 24.04に [ROS 2 Jazzyの公式手順](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) に従ってROSのaptリポジトリとROS 2を設定します。その後、次の依存をインストールします。

```bash
sudo apt update
sudo apt install \
  git ros-jazzy-ros-gz \
  ros-jazzy-slam-toolbox ros-jazzy-nav2-amcl \
  ros-jazzy-nav2-map-server ros-jazzy-nav2-lifecycle-manager \
  ros-jazzy-robot-localization \
  python3-numpy python3-scipy python3-yaml python3-pil python3-psutil \
  python3-pyside2.qtcore python3-pyside2.qtgui python3-pyside2.qtwidgets python3-pyside2.qttest \
  fonts-noto-cjk
```

`ros-jazzy-ros-gz`はROS 2とGazeboの連携用パッケージです。JazzyとHarmonicの組合せ、ROSリポジトリからの導入については [Gazebo公式のインストール案内](https://gazebosim.org/docs/harmonic/ros_installation/) を参照してください。

Pythonは `/usr/bin/python3` を使用します。ROSのaptパッケージと同じPython環境に揃えます。

PySide2はUbuntuのapt版を使用します。QtCore・QtGui・QtWidgetsは画面の実行用、QtTestは操作の回帰試験用です。MITは本リポジトリの独自部分に適用し、PySide2／Qt5などの外部依存は各ライセンスに従います。[第三者ソフトの説明](../THIRD_PARTY_NOTICES.md) を参照してください。

## クローンと診断

WSLのUbuntuターミナルで実行します。

```bash
cd ~
git clone https://github.com/futurecortexlabs/FCTX-ros2-gazebo-autonomous-cart.git
cd FCTX-ros2-gazebo-autonomous-cart
source /opt/ros/jazzy/setup.bash
python3 scripts/check_environment.py
```

ROS・Python依存、7コースの保存地図、画面環境を確認できます。結果は `logs/environment_check.json` に保存されます。依存が不足していると診断は終了コード1になります。`maps` の各項目が `ready` であることも確認してください。

以降のコマンドはこのクローンフォルダから実行します。起動スクリプトは自分自身の場所を基準に動くため、開発者と同じユーザー名やフォルダ名にする必要はありません。

## 初回起動

```bash
bash run_courses.sh --course warehouse_course.sdf --pose-source localization --start
```

Gazebo画面と操作画面が開きます。位置推定と地図の準備ができるまで待ち、「読み込み済み」を確認します。開始時は手動・停止です。

1. 手動操作はWASD／矢印キー、または画面ボタン。
2. 自由走行は「自動回避」。
3. 指定場所への移動は右側の地図をクリックして「この目的地へ移動」。
4. 「停止ロック」またはSpaceで停止。
5. 「コース終了」でGazeboを終了。操作画面は残り、別コースを選べます。

操作の詳細は [RUNBOOK](../RUNBOOK.md) と [配送ガイド](../MISSION_GUIDE.md) を参照してください。

## 画面とフォント

WSLではWSLgが使える状態で起動してください。`DISPLAY`が空の場合、診断の `display_configured` はfalseになります。

日本語表示にはシステムのNoto Sans CJK JPを利用できます。ローカルにMeiryoがある場合は既存設定が優先します。Windowsフォントの `.ttc` はリポジトリに含めません。

WSLのD3D12ライブラリがある場合は、Gazebo終了時のクラッシュ対策が起動時に適用されます。描画設定を指定する場合は、起動前に `GALLIUM_DRIVER` や `MESA_D3D12_DEFAULT_ADAPTER_NAME` を環境に設定できます。実際のGPU・ドライバで確認してください。

## 実行データ

地図保存は `maps/`、ルート保存は `routes/` を更新します。再現用に残したい地図やルートはGitで管理してください。通常ログ・PID・画像は `logs/` に出力します。

既存のWindows開発フォルダとWSL実行フォルダは別コピーです。新規利用では上記のクローンを使います。両方を編集する開発環境では、実行対象のコピーへ変更を反映してから検証してください。
