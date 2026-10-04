#!/usr/bin/env python3
# ファイルの役割: 基本コースのSDFを読み、衝突判定付きの箱2個とポール1個を追加して別のSDFへ保存する。
"""Add visible, collidable boxes and pylons to the existing course."""
import math
import xml.etree.ElementTree as E
from build_loop_world import ROOT, tag, shape


# 基本コースのSDFを読み、衝突判定付きの箱2個とポール1個を追加して別のSDFへ保存する。 起動から終了処理までをまとめる入口。
def main():
    tree = E.parse(ROOT/'worlds/loop_course.sdf')
    world = tree.getroot().find('world')
    # 5cm占有格子でも.42mの中継余裕を確保し、壁との物理重なりは避ける。
    for name, radius, angle, kind in [
        ('obstacle_box_inner', 3.4, 1.2, 'box'),
        ('obstacle_pylon_outer', 5.6, 3.2, 'cylinder'),
        ('obstacle_box_return', 3.4, 5.0, 'box'),
    ]:
        model = tag(world, 'model', name=name)
        tag(model, 'static', 'true')
        tag(model, 'pose', f'{radius*math.cos(angle)} {radius*math.sin(angle)} 0 0 0 {angle}')
        link = tag(model, 'link', name='obstacle_link')
        dims = '0.6 0.6 0.9' if kind == 'box' else (.30, .9)
        shape(link, name+'_collision', kind, dims, '0 0 .45 0 0 0')
        shape(link, name+'_visual', kind, dims, '0 0 .45 0 0 0', '1 .30 .035 1')
        stripe = '.61 .61 .12' if kind == 'box' else (.305, .12)
        for z in (.3, .65):
            shape(link, name+'_stripe_'+str(z), kind, stripe, f'0 0 {z} 0 0 0', '.98 .98 .95 1')
    E.indent(tree)
    out = ROOT/'worlds/obstacle_course.sdf'
    tree.write(out, encoding='utf-8', xml_declaration=True)
    print(out)


if __name__ == '__main__':
    main()
