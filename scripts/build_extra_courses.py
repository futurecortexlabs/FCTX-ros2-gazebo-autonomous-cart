# ファイルの役割: 基本コースの台車とセンサーを再利用し、楕円コースと棚のある倉庫コースを生成する。
"""Generate an oval track and an enclosed warehouse from the cart world."""
import math
import xml.etree.ElementTree as E
from build_loop_world import ROOT, tag, shape, localization_landmarks


# 基本SDFから元のコース装飾・壁を除き、台車とセンサーを残して新しいコースの土台にする。
def base():
    tree = E.parse(ROOT/'worlds/loop_course.sdf')
    w = tree.getroot().find('world')
    for m in list(w.findall('model')):
        if m.get('name') in ('inner_wall', 'outer_wall', 'island', 'route_markings') or m.get('name','').startswith('obstacle_localization_'):
            w.remove(m)
    return tree, w


# 静的モデルとlinkを追加する。名前は壁・障害物の識別にも使われる。
def wall(w, name):
    m = tag(w, 'model', name=name)
    tag(m, 'static', 'true')
    return tag(m, 'link', name='wall')


# 同じ寸法と位置で衝突用と表示用の箱を追加する。
def box(link, name, size, pose, color='.8 .87 .94 1'):
    shape(link, name, 'box', size, pose)
    shape(link, name+'_visual', 'box', size, pose, color)


# 基本コースの台車とセンサーを再利用し、楕円コースと棚のある倉庫コースを生成する。 起動から終了処理までをまとめる入口。
def main():
    # 既存の台車・センサーを共通部として使い、コース形状だけを作り直す。
    tree, w = base()
    for name, a, b in [('inner_wall', 4.5, 3.), ('outer_wall', 7.5, 6.)]:
        link = wall(w, name)
        for i in range(120):
            t = i*math.tau/120
            tangent = math.atan2(b*math.cos(t), -a*math.sin(t))
            length = math.hypot(a*math.sin(t), b*math.cos(t))*math.tau/120+.035
            box(link, f'wall_{i}', f'{length} .16 .9',
                f'{a*math.cos(t)} {b*math.sin(t)} .45 0 0 {tangent}',
                '.08 .55 .68 1' if name.startswith('inner') else '.84 .88 .92 1')
    w.find("model[@name='loop_cart']/pose").text = f'6 0 .18 0 0 {math.pi/2}'
    localization_landmarks(w,7.5,6.)
    E.indent(tree)
    tree.write(ROOT/'worlds/oval_course.sdf', encoding='utf-8', xml_declaration=True)
    # 既存の台車・センサーを共通部として使い、コース形状だけを作り直す。
    tree, w = base()
    link = wall(w, 'outer_wall')
    for i, (size, pose) in enumerate([
        ('14 .16 .9', '0 5 .45 0 0 0'), ('14 .16 .9', '0 -5 .45 0 0 0'),
        ('.16 10 .9', '7 0 .45 0 0 0'), ('.16 10 .9', '-7 0 .45 0 0 0')]):
        box(link, f'wall_{i}', size, pose)
    for i, (x, y) in enumerate([(-2.5, -1.4), (-2.5, 1.4), (2.5, -1.4), (2.5, 1.4)]):
        link = wall(w, f'obstacle_shelf_{i}')
        box(link, f'obstacle_shelf_{i}', '1.8 .65 1.2', f'{x} {y} .6 0 0 0', '.95 .45 .08 1')
    w.find("model[@name='loop_cart']/pose").text = '0 -3.4 .18 0 0 0'
    E.indent(tree)
    tree.write(ROOT/'worlds/warehouse_course.sdf', encoding='utf-8', xml_declaration=True)


if __name__ == '__main__':
    main()
