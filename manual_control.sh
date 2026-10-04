#!/usr/bin/env bash
# 既存のROSシミュレーションへ接続する操作パネルだけを起動する。
# コマンド失敗を検出して終了し、不完全な起動を続けない。
set -eo pipefail
# 呼び出し元に関係なくプロジェクトのフォルダで実行する。
cd "$(dirname "$(readlink -f "$0")")"
export LANG=C.UTF-8
export LC_ALL=C.UTF-8
export PYTHONUTF8=1
# プロジェクトの日本語フォント設定を利用する。
export FONTCONFIG_FILE="$PWD/fonts/fonts.conf"
# ROS 2の実行ファイル・ライブラリを使える環境を読み込む。
source /opt/ros/jazzy/setup.bash
# 実験用のROS通信をドメイン42にまとめる。
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export QT_QPA_PLATFORM=xcb
exec /usr/bin/python3 "$PWD/scripts/manual_panel.py"
