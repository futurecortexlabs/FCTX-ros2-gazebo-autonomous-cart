# FCTX-公開検証記録

公開する機能と検証実績の根拠を保存しています。条件、測定値の意味、再実行方法は [検証結果と適用範囲](../../docs/VALIDATION.md) を参照してください。

| 検証 | 統合記録と補足 |
|---|---|
| 配布する7地図 | [shipped_map_coverage.json](shipped_map_coverage.json)：観測率と通行可能率。[map_generation_provenance.json](map_generation_provenance.json)：配布地図の生成設定・SHA |
| 全7コースAMCL配送 | [saved_routes_summary.json](saved_routes_summary.json)：98地点、接触、停止、地図・ルートSHA。各コースの原記録へ参照あり |
| 共通最終設定のSLAM | [final_slam_regression_summary.json](final_slam_regression_summary.json)：105検査地点、誤差、接触、正常終了。原記録とソース・所有プロセス確認へ参照あり |
| 倉庫連続配送 | [endurance_validation.json](endurance_validation.json)：30分51秒、132地点。[endurance_source_verification.json](endurance_source_verification.json)：使用ソース・配布地図の照合 |
| 動的障害物・安全停止 | [dynamic_route_validation.json](dynamic_route_validation.json)、[localized_ui_validation.json](localized_ui_validation.json) |
| 終了・コース切替 | [shutdown_timing_validation.json](shutdown_timing_validation.json)、[final_shutdown_verification.json](final_shutdown_verification.json)、[lifecycle_shutdown_verification.json](lifecycle_shutdown_verification.json)、[shutdown_input_targeted_verification.json](shutdown_input_targeted_verification.json)、[final_course_switch_verification.json](final_course_switch_verification.json) |
| 起動復旧・描画 | [startup_recovery_validation.json](startup_recovery_validation.json)、[software_renderer_final_verification.json](software_renderer_final_verification.json)。描画は同じWSLマシンでのllvmpipe試験 |
| 低速のSLAM更新 | [low_speed_slam_comparison.json](low_speed_slam_comparison.json)：0.05m/s、25秒の比較と0.5m目標。全コース試験とは別 |
| 検証器の終了判定 | [final_validator_cleanup_units.json](final_validator_cleanup_units.json)、[validator_shutdown_integration.json](validator_shutdown_integration.json) |
| 単体試験・初回診断 | [final_release_validation.json](final_release_validation.json)：17入口、実行器6条件、全7地図。[公開用整理後の再確認](../20261004/release_static_validation.json) |

SLAM回帰で生成した地図は試験コピー用です。配布地図・ルートはAMCL配送で確認したものを維持しています。正解位置は外部評価とGUIの速度表示に使用し、SLAM/AMCLの制御・安全・位置推定には入力しません。

結果のJSONには試験当時の成否、日時、設定、ソースSHAを保持しています。最終SLAM走行と終了判定を補強した検証器の版差、円形・大型倉庫の検証器・観測器のOS終了コード未取得も記録に残します。個人ディレクトリは `<workspace>` / `<home>` として匿名化済みです。元記録のSHAは匿名化前の記録を指し、公開用JSONの現在のファイル全体のSHAとは異なります。
