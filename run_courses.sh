#!/usr/bin/env bash
# コース選択付きの操作画面を起動する。--startを渡すと初期選択コースも読み込む。
# コマンド失敗を検出して終了し、不完全な起動を続けない。
set -eo pipefail
# 呼び出し元に関係なくプロジェクトのフォルダで実行する。
cd "$(dirname "$(readlink -f "$0")")"
# ROS 2の実行ファイル・ライブラリを使える環境を読み込む。
source /opt/ros/jazzy/setup.bash
# 実験用のROS通信をドメイン42にまとめる。
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export LANG=C.UTF-8
export LC_ALL=C.UTF-8
# プロジェクトの日本語フォント設定を利用する。
export FONTCONFIG_FILE="$PWD/fonts/fonts.conf"
exec /usr/bin/python3 scripts/course_selector.py "$@"
