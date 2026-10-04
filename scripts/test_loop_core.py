# ファイルの役割: ROSやGazeboなしで、近接停止・無効測定・車体サイズを考慮した距離計算を検証する。
import math
import unittest
from loop_core import safe_command, scan_distances, clearance_lower_bound


# 関連する検証ケースをまとめ、失敗条件をassertで確認するテストクラス。
class LoopTests(unittest.TestCase):
    # 近すぎる障害物では停止し、少し離れた障害物では速度を下げることを確認する。
    def test_obstacle_stop_and_slowdown(self):
        self.assertEqual(safe_command(.65, .15, .7, 2)[:2], (0, 0))
        self.assertEqual(safe_command(.65, .15, 1.0, .9)[:2], (0, 0))
        self.assertLess(safe_command(.65, .15, 1.4, 1.3)[0], .65)

    # 無効な測距を安全な空間と誤解せず、判定不能として返すことを確認する。
    def test_invalid_scan_fails_closed(self):
        self.assertIsNone(scan_distances([float('inf')]*720, -math.pi, math.tau/719, .06, 15))
        self.assertIsNone(scan_distances([float('nan')]*720, -math.pi, math.tau/719, .06, 15))

    # 台車中心ではなく車体全体を考慮した壁との余裕になっていることを確認する。
    def test_clearance_includes_robot_size(self):
        self.assertGreater(clearance_lower_bound(4.5, 0), .79)
        self.assertLess(clearance_lower_bound(5.5, 0), 0)


if __name__ == '__main__':
    unittest.main()
