"""SLAM/AMCLの占有地図を経路計画に使用。未知領域は障害物として扱う。"""
import math
import numpy as np
from scipy.ndimage import distance_transform_edt
from goal_navigation import NavigationMap

class OccupancyNavigationMap(NavigationMap):
    def __init__(self,msg):
        info=msg.info;self.resolution=info.resolution
        if self.resolution<=0 or info.width*info.height!=len(msg.data):raise ValueError('地図サイズが不正です')
        if abs(info.origin.orientation.z)>1e-6:raise ValueError('回転した占有地図は未対応です')
        self.xmin=info.origin.position.x+self.resolution/2;self.ymin=info.origin.position.y+self.resolution/2
        self.xs=self.xmin+np.arange(info.width)*self.resolution;self.ys=self.ymin+np.arange(info.height)*self.resolution
        self.xmax=float(self.xs[-1]);self.ymax=float(self.ys[-1]);self.dynamic=[]
        grid=np.asarray(msg.data,dtype=np.int8).reshape(info.height,info.width)
        self.known_free=(grid>=0)&(grid<50)
        self.observed_occupied=grid>=50
        # 地図の端にも障害物を置き、未知の外側を経路に使用しない。
        padded=np.pad(self.known_free,1,constant_values=False)
        self.static_clearances=distance_transform_edt(padded)[1:-1,1:-1]*self.resolution-.62-self.resolution*.71
        self.clearances=self.static_clearances.copy();self.free=self.clearances>=.28
        self.geometry=GridGeometry(self)
    def distances(self,x,y):
        x,y=np.broadcast_arrays(np.asarray(x,dtype=float),np.asarray(y,dtype=float))
        ix=np.rint((x-self.xmin)/self.resolution).astype(int);iy=np.rint((y-self.ymin)/self.resolution).astype(int)
        valid=(ix>=0)&(iy>=0)&(ix<len(self.xs))&(iy<len(self.ys))
        result=np.full(x.shape,-.62);result[valid]=self.clearances[iy[valid],ix[valid]]
        return result

class GridGeometry:
    shapes=[]
    def __init__(self,map):self.map=map
    def clearance(self,x,y):
        row,col=self.map.cell((x,y))
        if not 0<=row<len(self.map.ys) or not 0<=col<len(self.map.xs):return -.62
        return float(self.map.static_clearances[row,col])

    def observed_obstacle_near(self,x,y,distance):
        """未知セルを証拠にせず、観測済み占有セルの近傍だけを照合する。"""
        row,col=self.map.cell((x,y))
        radius=math.ceil((distance+self.map.resolution*.71)/self.map.resolution)
        bottom,top=max(0,row-radius),min(len(self.map.ys),row+radius+1)
        left,right=max(0,col-radius),min(len(self.map.xs),col+radius+1)
        if bottom>=top or left>=right:return False
        ys,xs=np.nonzero(self.map.observed_occupied[bottom:top,left:right])
        if not len(xs):return False
        distances=np.hypot(xs+left-col,ys+bottom-row)*self.map.resolution-self.map.resolution*.71
        return bool(np.any(distances<distance))

    def clearances(self,x,y):
        """静的な距離場をまとめて参照する。未知・範囲外は従来どおり通行不可。"""
        x,y=np.broadcast_arrays(np.asarray(x,dtype=float),np.asarray(y,dtype=float))
        col=np.rint((x-self.map.xmin)/self.map.resolution).astype(int)
        row=np.rint((y-self.map.ymin)/self.map.resolution).astype(int)
        valid=(row>=0)&(col>=0)&(row<len(self.map.ys))&(col<len(self.map.xs))
        result=np.full(x.shape,-.62);result[valid]=self.map.static_clearances[row[valid],col[valid]]
        return result
