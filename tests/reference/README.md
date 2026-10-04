# FCTX-最適化回帰試験の比較実装

`dynamic_obstacles_preoptimization.py` は、2026年9月19日の最適化前に保存した
`before_optimization_20260919_035108/dynamic_obstacles.py` の内容を変更せずにコピーしたものです。

SHA-256: `88ceca221156b031809a12789d5e8218085866e61db0201a2f01f4b4910b09e9`

`scripts/test_optimization.py` は、この固定された過去実装と現行の
`scripts/dynamic_obstacles.py` に同じ入力を与え、障害物セルと更新判定の一致を検証します。
比較対象をリポジトリに含めることで、開発環境のバックアップや実行ログがないクローンでも試験できます。
このファイルは検証資料であり、通常の走行制御では読み込みません。

比較の基準を維持するため、この過去実装を現行実装に合わせて変更しないでください。
