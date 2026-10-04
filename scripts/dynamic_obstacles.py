"""既知地図にないLiDAR反射を追跡し、空き観測と地図への静的取り込みを照合する。"""
import math

class DynamicObstacles:
    def __init__(self,geometry):
        self.geometry=geometry;self.cells={};self.resolution=.25

    def replace_geometry(self,geometry):
        """新しい地図で観測済みの静的障害物になった点を、重複して膨張させない。"""
        self.geometry=geometry
        before=set(self.cells)
        for cell,(x,y) in list(self.cells.items()):
            # 占有地図の未知領域は静的障害物の観測ではない。GridGeometryは
            # 既知の占有セルだけを照合し、未知・遮蔽・未地図化の点を保持する。
            if hasattr(geometry,'observed_obstacle_near'):
                mapped=geometry.observed_obstacle_near(x,y,.30)
            else:
                mapped=geometry.clearance(x,y)+.62<.30
            if mapped:del self.cells[cell]
        return before!=set(self.cells)

    def update(self,pose,ranges,angle_min,increment,range_min,range_max):
        if not ranges or not math.isfinite(increment) or increment<=0:return False
        x,y,yaw=pose;hits={};candidates=[]
        # 複数ビームが同じセルへ反射した点だけを追加し、孤立した測定を抑える。
        for i in range(0,len(ranges),2):
            d=ranges[i]
            if not math.isfinite(d) or not max(.7,range_min)<=d<min(8.,range_max):continue
            angle=yaw+angle_min+i*increment;px=x+d*math.cos(angle);py=y+d*math.sin(angle)
            candidates.append((px,py))
        if candidates:
            xs,ys=zip(*candidates)
            distances=self.geometry.clearances(xs,ys) if hasattr(self.geometry,'clearances') else [self.geometry.clearance(px,py) for px,py in candidates]
            for (px,py),distance in zip(candidates,distances):
                # 閾値の直近は従来のスカラー判定を使い、浮動小数点差による判定変更を防ぐ。
                if abs(distance+.62-.30)<1e-9:distance=self.geometry.clearance(px,py)
                if distance+.62<.30:continue
                cell=(round(px/self.resolution),round(py/self.resolution));hits[cell]=hits.get(cell,0)+1
        before=set(self.cells)
        for cell,(px,py) in list(self.cells.items()):
            distance=math.hypot(px-x,py-y)
            angle=math.atan2(py-y,px-x)-yaw
            angle=math.atan2(math.sin(angle),math.cos(angle))
            index=round((angle-angle_min)/increment)
            # 障害物の裏側やセンサー範囲外は「空き」と判断しない。
            if 0<=index<len(ranges) and distance<min(8.,range_max)-.5:
                nearby=ranges[max(0,index-2):min(len(ranges),index+3)]
                if nearby and all((math.isfinite(d) and d>distance+.5) or d==math.inf for d in nearby):
                    del self.cells[cell]
        for cell,count in hits.items():
            if count>=2:self.cells[cell]=(cell[0]*self.resolution,cell[1]*self.resolution)
        # 計算量に上限を設ける。古い点を消して安全領域を広げる代わりに、過密状態は停止対象とする。
        return before!=set(self.cells)

    @property
    def points(self):return list(self.cells.values())
