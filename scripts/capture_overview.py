#!/usr/bin/env python3
# ファイルの役割: Gazeboの俯瞰カメラ画像をROSで受信し、RGBの画素からPNGファイルを作って保存する。
"""Capture the actual Gazebo overhead sensor image as a PNG."""
import struct
import time
import zlib
from pathlib import Path
import rclpy
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data


# Gazeboの俯瞰カメラ画像をROSで受信し、RGBの画素からPNGファイルを作って保存する。 起動から終了処理までをまとめる入口。
def main():
    rclpy.init()
    node = rclpy.create_node('capture_loop_overview')
    images = []
    node.create_subscription(Image, '/loop/overview', images.append, qos_profile_sensor_data)
    until = time.monotonic()+20
    while not images and time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=.2)
    if not images:
        raise RuntimeError('No overhead image received')
    im = images[-1]
    assert im.encoding in ('rgb8', 'bgr8'), im.encoding
    data = bytes(im.data)
    rows = []
    for y in range(im.height):
        row = data[y*im.step:y*im.step+im.width*3]
        if im.encoding == 'bgr8':
            row = bytes(v for x in range(0, len(row), 3) for v in (row[x+2], row[x+1], row[x]))
        rows.append(b'\x00'+row)
    # PNGのチャンク長・種類・内容・CRCを結合し、PNG形式の1ブロックを作る。
    def chunk(kind, payload):
        return struct.pack('!I',len(payload))+kind+payload+struct.pack('!I', zlib.crc32(kind+payload)&0xffffffff)
    png = b'\x89PNG\r\n\x1a\n'
    png += chunk(b'IHDR', struct.pack('!2I5B', im.width, im.height, 8, 2, 0, 0, 0))
    png += chunk(b'IDAT', zlib.compress(b''.join(rows))) + chunk(b'IEND', b'')
    output = Path(__file__).resolve().parents[1]/'logs/loop_overview.png'
    output.write_bytes(png)
    print(output)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
