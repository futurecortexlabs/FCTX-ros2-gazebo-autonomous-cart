# ファイルの役割: 仮想LiDARの距離列を使い、直進、左右対称性、近接停止、空いた側への旋回を検証する。
"""Synthetic LiDAR checks including mirrored clockwise/counterclockwise starts."""
import math
import unittest
import numpy as np
from obstacle_avoidance import ObstacleAvoidance


# 円形の内外壁へ飛ばした仮想光線から距離列を作り、実センサーなしで回避器を試せるようにする。
def circle_scan(x, y, yaw):
    angles = np.linspace(-math.pi, math.pi, 360)
    dx, dy = np.cos(angles+yaw), np.sin(angles+yaw)
    dot = x*dx+y*dy
    r2 = x*x+y*y
    distances = np.full(angles.shape, 15.)
    for radius in (3.08, 5.92):
        disc = dot*dot-r2+radius*radius
        valid = disc>=0
        for sign in (-1, 1):
            t = -dot+sign*np.sqrt(np.maximum(disc, 0))
            distances = np.minimum(distances, np.where(valid & (t>0), t, 15.))
    return distances


# 関連する検証ケースをまとめ、失敗条件をassertで確認するテストクラス。
class AvoidanceTests(unittest.TestCase):
    # 十分に開けた測距では不要な旋回をせず前進を選ぶことを確認する。
    def test_open_space_keeps_heading(self):
        p = ObstacleAvoidance()
        v, w = p.command([8.]*360, -math.pi, math.tau/359, .06, 15.)
        self.assertGreater(v, 0)
        self.assertAlmostEqual(w, 0)

    # 開始方向を反転したとき旋回も左右反転し、一方の周回方向へ固定されないことを確認する。
    def test_mirrored_heading_does_not_force_one_lap_direction(self):
        outputs = []
        for yaw in (math.pi/2, -math.pi/2):
            p = ObstacleAvoidance()
            outputs.append(p.command(circle_scan(4.5,0,yaw), -math.pi, math.tau/359, .06,15.))
        self.assertGreater(outputs[0][0], .1)
        self.assertGreater(outputs[1][0], .1)
        self.assertAlmostEqual(outputs[0][0], outputs[1][0])
        self.assertAlmostEqual(outputs[0][1], -outputs[1][1])

    # 無効な測距や極端な近接が停止指令になることを確認する。
    def test_invalid_and_close_returns_stop(self):
        p = ObstacleAvoidance()
        for scan in ([float('nan')]*360, [.7]*360):
            self.assertEqual(p.command(scan,-math.pi,math.tau/359,.06,15.), (0.,0.))

    # 正面が塞がったとき、左右のうち余裕がある側へのその場旋回を選ぶことを確認する。
    def test_front_wall_chooses_open_side(self):
        p = ObstacleAvoidance()
        angles = np.linspace(-math.pi,math.pi,360)
        scan = np.where(np.abs(angles)<.6, 1.0, np.where(angles<0,1.1,3.))
        v,w = p.command(scan,-math.pi,math.tau/359,.06,15.)
        self.assertEqual(v,0.)
        self.assertGreater(w,0.)


if __name__ == '__main__':
    unittest.main()
