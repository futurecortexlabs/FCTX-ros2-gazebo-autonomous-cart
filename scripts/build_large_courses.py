"""広い楕円・大型倉庫・交互障害壁コースを生成する。既存4コースは変更しない。"""
import math
import xml.etree.ElementTree as E
from build_extra_courses import base, wall, box
from build_loop_world import ROOT, localization_landmarks


def save(tree, world, filename, pose):
    # 全景が見える距離へGUIと記録用カメラを移す。
    world.find("model[@name='loop_cart']/pose").text = pose
    for camera in world.findall('.//camera_pose'):
        camera.text = '21 -28 32 0 0.74 2.21'
    camera = world.find("model[@name='overview_camera']/link/pose")
    if camera is not None:
        camera.text = '21 -28 36 0 0.80 2.21'
    E.indent(tree)
    tree.write(ROOT/'worlds'/filename, encoding='utf-8', xml_declaration=True)


def enclosure(world, width, height):
    # 壁の名前を維持し、既存の接触監視と地図生成に対応させる。
    link = wall(world, 'outer_wall')
    for i, (size, pose) in enumerate([
        (f'{width} .16 1', f'0 {height/2} .5 0 0 0'),
        (f'{width} .16 1', f'0 {-height/2} .5 0 0 0'),
        (f'.16 {height} 1', f'{width/2} 0 .5 0 0 0'),
        (f'.16 {height} 1', f'{-width/2} 0 .5 0 0 0')]):
        box(link, f'wall_{i}', size, pose)


def main():
    tree, w = base()
    # 外周24×16m、内周16×8m。従来の楕円より広い走行帯を確保する。
    for name, a, b in [('inner_wall',8.,4.),('outer_wall',12.,8.)]:
        link = wall(w,name)
        for i in range(160):
            t=i*math.tau/160
            yaw=math.atan2(b*math.cos(t),-a*math.sin(t))
            length=math.hypot(a*math.sin(t),b*math.cos(t))*math.tau/160+.035
            box(link,f'wall_{i}',f'{length} .16 1',f'{a*math.cos(t)} {b*math.sin(t)} .5 0 0 {yaw}', '.12 .55 .65 1')
    localization_landmarks(w,12.,8.)
    save(tree,w,'large_oval_course.sdf',f'10 0 .18 0 0 {math.pi/2}')

    tree,w=base();enclosure(w,26,20)
    # 3列×3段の棚の間に、台車が通行できる通路を設ける。
    for i,(x,y) in enumerate(( (x,y) for x in (-7,0,7) for y in (-5,0,5) )):
        link=wall(w,f'obstacle_shelf_{i}')
        box(link,f'obstacle_shelf_{i}','3 1.4 1.4',f'{x} {y} .7 0 0 0','.95 .48 .10 1')
    save(tree,w,'large_warehouse_course.sdf','-10 -7.5 .18 0 0 0')

    tree,w=base();enclosure(w,24,16)
    # 上下交互の壁で迂回を作る。各壁の先端に6mの開口を残す。
    for i,x in enumerate((-6,0,6)):
        y=-3 if i%2==0 else 3
        link=wall(w,f'obstacle_barrier_{i}')
        box(link,f'obstacle_barrier_{i}','.7 10 1.2',f'{x} {y} .6 0 0 0','.9 .32 .10 1')
    save(tree,w,'slalom_course.sdf','-10 -5 .18 0 0 1.57079632679')


if __name__=='__main__':main()
