# FCTX-PySide2移行後の検証 / PySide2 migration checks

2026-10-05、Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic / Python 3.12 / PySide2 5.15.13 / Qt 5.15.13で確認しました。各JSONは実行時のソースSHAを保持します。

- [単体試験と初回診断](mit_release_validation.json)：18入口、実行器の成功・失敗・タイムアウト6条件、別コピーでの依存と7地図の診断。追加Qt2件は、地図の上下方向・画像バッファ寿命と、実QProcessの出力読込・終了・破棄・再実行を確認。
- [Gazebo・AMCLのGUI統合](mit_gui_integration.json)：倉庫コースの地図表示、AMCL停止注入での安全停止・復帰、ルート保存・読込、2地点配送、接触0、完了後の実停止。終了時はGazebo一時停止、所有launchの終了コード0、SIGINTのみ、所有プロセス残存0を確認。

通常運転と異なるROS domain / Gazebo partition、隔離コピーで実行し、付属地図・ルートは更新していません。公開記録ではコースの絶対パスを相対パスに置換しています。画像と生ログは配布に含めません。

These are migration checks in the existing WSL environment. They cover the 18 unit-test entry points, six runner cases, fresh-copy diagnostics, and a short warehouse AMCL GUI integration with interruption, delivery, and normal shutdown. They do not repeat the earlier seven-course SLAM/AMCL tours, 30-minute endurance, or visible-window rendering tests. Those earlier results used PyQt5 and retain their original source hashes.

再実行手順と範囲は[検証資料](../../docs/VALIDATION.md)に記載しています。全7コースのSLAM/AMCL巡回・30分連続配送・表示ウィンドウ付き描画は今回再実行していません。以前の結果はPyQt5を使用した試験時点の記録として保持しています。
