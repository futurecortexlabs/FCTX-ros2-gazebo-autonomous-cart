#!/usr/bin/env bash
# 共通入口へ手動モードを指定し、台車を停止待機の状態で開始する。
# コマンド失敗を検出して終了し、不完全な起動を続けない。
set -eo pipefail
# 呼び出し元に関係なくプロジェクトのフォルダで実行する。
cd "$(dirname "$(readlink -f "$0")")"
exec bash run_loop.sh mode:=manual "$@"
