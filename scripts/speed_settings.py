# ファイルの役割: 操作画面・回避器・安全ゲートで共有する速度上限とプリセット。並進はm/s、旋回はrad/s。
"""Shared speed limits for UI, local planner and final command gate."""
MAX_FORWARD_SPEED = 1.50
MAX_REVERSE_SPEED = 0.80
MAX_TURN_SPEED = 1.50
MAX_AUTO_SPEED = 1.50
DEFAULT_AUTO_SPEED = 0.45
PRESETS = {
    'low': (0.20, 0.15, 0.40, 0.20),
    'normal': (0.40, 0.25, 0.65, 0.45),
    'fast': (1.00, 0.50, 1.00, 0.90),
}
