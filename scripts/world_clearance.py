# ファイルの役割: 選択したSDFの衝突形状を読む。距離記録と目的地移動の地図として使用する。
"""Static collision geometry for telemetry and known-map goal navigation."""
import math
import numpy as np
import xml.etree.ElementTree as E


# SDFから読み込んだ静的形状の距離計算器。
class WorldClearance:
    # 静的な壁・障害物のSDFを解析し、model/link/collisionの平面変換を合成して箱・円柱を記録する。
    def __init__(self, path, walls_only=False):
        self.shapes = []
        for model in E.parse(path).getroot().find('world').findall('model'):
            if model.findtext('static') != 'true':
                continue
            if not any(word in model.get('name', '') for word in ('wall', 'obstacle_')):
                continue
            if walls_only and 'wall' not in model.get('name', ''):
                continue
            mx, my, _, _, _, ma = map(float, model.findtext('pose', '0 0 0 0 0 0').split())
            for link in model.findall('link'):
                lx, ly, _, _, _, la = map(float, link.findtext('pose', '0 0 0 0 0 0').split())
                for c in link.findall('collision'):
                    x, y, _, _, _, a = map(float, c.findtext('pose', '0 0 0 0 0 0').split())
                    x, y = lx+x*math.cos(la)-y*math.sin(la), ly+x*math.sin(la)+y*math.cos(la)
                    x, y = mx+x*math.cos(ma)-y*math.sin(ma), my+x*math.sin(ma)+y*math.cos(ma)
                    g = c.find('geometry')
                    if g.find('box') is not None:
                        sx, sy, _ = map(float, g.findtext('box/size').split())
                        self.shapes.append((x, y, a+la+ma, sx/2, sy/2, 'box'))
                    elif g.find('cylinder') is not None:
                        self.shapes.append((x, y, 0., float(g.findtext('cylinder/radius')), 0., 'circle'))

    # ワールド座標の台車中心から各箱・円柱への最短距離を求め、車体を囲む0.62mを引く。
    def clearance(self, x, y):
        best = math.inf
        for cx, cy, angle, hx, hy, kind in self.shapes:
            dx, dy = x-cx, y-cy
            if kind == 'circle':
                distance = max(0., math.hypot(dx, dy)-hx)
            else:
                u, v = dx*math.cos(angle)+dy*math.sin(angle), -dx*math.sin(angle)+dy*math.cos(angle)
                distance = math.hypot(max(0., abs(u)-hx), max(0., abs(v)-hy))
            best = min(best, distance)
        return best-.62


    def clearances(self,x,y):
        """複数のLiDAR反射点を一括計算し、Pythonでの点×壁の二重ループを避ける。"""
        x,y=np.broadcast_arrays(np.asarray(x,dtype=float),np.asarray(y,dtype=float))
        if not self.shapes:return np.full(x.shape,np.inf)
        if not hasattr(self,'_vector_shapes'):
            self._vector_shapes=np.array([(cx,cy,math.cos(a),math.sin(a),hx,hy,kind=='circle') for cx,cy,a,hx,hy,kind in self.shapes],dtype=float).T
        cx,cy,ca,sa,hx,hy,circle=(v[:,None] for v in self._vector_shapes)
        dx=x.reshape(1,-1)-cx;dy=y.reshape(1,-1)-cy
        u=dx*ca+dy*sa;v=-dx*sa+dy*ca
        distances=np.hypot(np.maximum(np.abs(u)-hx,0.),np.maximum(np.abs(v)-hy,0.))
        if np.any(circle):distances=np.where(circle,np.maximum(np.hypot(dx,dy)-hx,0.),distances)
        return (np.min(distances,axis=0)-.62).reshape(x.shape)
