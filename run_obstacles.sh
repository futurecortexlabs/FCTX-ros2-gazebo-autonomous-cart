#!/usr/bin/env bash
# 障害物付きSDFを共通入口へ渡して起動する。
# コマンド失敗を検出して終了し、不完全な起動を続けない。
set -eo pipefail
# 呼び出し元に関係なくプロジェクトのフォルダで実行する。
cd "$(dirname "$(readlink -f "$0")")"
exec bash run_loop.sh world:="$PWD/worlds/obstacle_course.sdf" "$@"
