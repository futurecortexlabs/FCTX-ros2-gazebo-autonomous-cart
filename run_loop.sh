#!/usr/bin/env bash
# Gazeboと制御ノード一式を起動する共通入口。追加引数はROS launchへそのまま渡す。
# コマンド失敗を検出して終了し、不完全な起動を続けない。
set -eo pipefail
# 呼び出し元に関係なくプロジェクトのフォルダで実行する。
cd "$(dirname "$(readlink -f "$0")")"
mkdir -p logs
# 排他ロック用のファイル記述子を確保する。
exec 9>logs/loop.lock
# 別のシミュレーションが動いている場合は二重起動しない。
if ! flock -n 9; then
  echo 'A loop simulation is already running. Close that window or stop it first.' >&2
  exit 1
fi
export LANG=C.UTF-8
export LC_ALL=C.UTF-8
export PYTHONUTF8=1
# プロジェクトの日本語フォント設定を利用する。
export FONTCONFIG_FILE="$PWD/fonts/fonts.conf"
# ROS 2の実行ファイル・ライブラリを使える環境を読み込む。
source /opt/ros/jazzy/setup.bash
# 実験用のROS通信をドメイン42にまとめる。
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
# Gazebo通信もこの実験用の区画へ分ける。
export GZ_PARTITION="${GZ_PARTITION:-ros2_gazebo_loop}"
export PYTHONUNBUFFERED=1
exec ros2 launch "$PWD/launch/loop_course.launch.py" "$@"
