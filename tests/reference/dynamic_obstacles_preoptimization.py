"""既知地図にないLiDAR反射を追跡する。障害物は観測で空きを確認した時だけ消す。"""
import math

class DynamicObstacles:
    def __init__(self,geometry):
        self.geometry=geometry;self.cells={};self.resolution=.25

    def update(self,pose,ranges,angle_min,increment,range_min,range_max):
        if not ranges or not math.isfinite(increment) or increment<=0:return False
        x,y,yaw=pose;hits={}
        # 複数ビームが同じセルへ反射した点だけを追加し、孤立した測定を抑える。
        for i in range(0,len(ranges),2):
            d=ranges[i]
            if not math.isfinite(d) or not max(.7,range_min)<=d<min(8.,range_max):continue
            angle=yaw+angle_min+i*increment;px=x+d*math.cos(angle);py=y+d*math.sin(angle)
            if self.geometry.clearance(px,py)+.62<.30:continue
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
