"""Local-metre GeoJSON and swept-footprint collision tests.

Every navigation vertex is a safe *pivot*. Lines use exact rectangular
translation sweeps, so a narrow straight corridor can be traversed when a
circumscribed disk alone would reject it. Rotation uses an all-angle envelope.
"""
from __future__ import annotations
from dataclasses import dataclass
import json, math
from pathlib import Path
import numpy as np
from shapely import affinity, make_valid
from shapely.geometry import Point, Polygon, LineString, box, shape, mapping
from shapely.ops import unary_union
from shapely.prepared import prep
from .models import Robot, PlanningError


def parts(g, kind='Polygon'):
    if g.is_empty:return []
    if g.geom_type==kind:return [g]
    return [p for x in getattr(g,'geoms',()) for p in parts(x,kind)]


def polygonal(g):
    return unary_union(parts(make_valid(g)))


def path_length(path):
    return sum(math.dist(a,b) for a,b in zip(path,path[1:]))


def dedup(path):
    out=[]
    for p in path:
        p=tuple(map(float,p))
        if not out or math.dist(p,out[-1])>1e-9:out.append(p)
    return out


def rectangle(p, yaw, length, width):
    c,s=math.cos(yaw),math.sin(yaw)
    return Polygon([(p[0]+c*x-s*y,p[1]+s*x+c*y)
                    for x,y in [(-length/2,-width/2),(length/2,-width/2),
                                (length/2,width/2),(-length/2,width/2)]])


def translation_sweep(p,q,robot,margin=0.):
    yaw=math.atan2(q[1]-p[1],q[0]-p[0])
    g=unary_union([rectangle(p,yaw,robot.length_m,robot.width_m),
                   rectangle(q,yaw,robot.length_m,robot.width_m)]).convex_hull
    return g.buffer(margin / math.cos(math.pi/128), quad_segs=32) if margin else g


def rotation_sweep(p,robot,margin=0.):
    # Circumscribed 128-gon, not an inscribed approximation to a circle.
    r=(robot.radius+margin)/math.cos(math.pi/128)
    return Point(p).buffer(r,quad_segs=32)

@dataclass
class FieldMap:
    field: object
    obstacles: list
    corridors: list
    metadata: dict
    original_geojson: dict

    @classmethod
    def load(cls,path):
        data=json.loads(Path(path).read_text(encoding='utf-8'))
        return cls.from_geojson(data)

    @classmethod
    def from_geojson(cls,data):
        if data.get('type')!='FeatureCollection':raise ValueError('Map must be a GeoJSON FeatureCollection in local metres')
        crs=str(data.get('crs','')).lower()
        if '4326' in crs or 'wgs84' in crs:raise ValueError('Longitude/latitude maps must be projected to local metres first')
        fields=[];obstacles=[];corridors=[]
        for f in data.get('features',[]):
            g=shape(f['geometry'])
            if g.is_empty:continue
            if not g.is_valid:raise ValueError('Invalid polygon map: repair it explicitly before planning')
            if g.geom_type not in ('Polygon','MultiPolygon'):raise ValueError('Only Polygon/MultiPolygon map features are supported')
            props=f.get('properties') or {}; kind=str(props.get('kind',props.get('type',''))).lower()
            if kind in ('field','boundary','farm'):fields.append(g)
            elif kind in ('obstacle','wall','building','tree'):obstacles.append(g)
            elif kind in ('corridor','gate','entrance'):corridors.append(g)
            else:raise ValueError(f'Unknown map feature kind: {kind!r}; refusing to ignore possible obstacles')
        if not fields:raise ValueError('No field polygon')
        field=unary_union(fields)
        if not math.isfinite(field.area) or field.area<=0:raise ValueError('Empty/nonfinite field')
        return cls(field,obstacles,corridors,data.get('metadata',{}),data)

class World:
    def __init__(self,field:FieldMap,robot:Robot,manual_setback=.8):
        robot.validate();self.map=field;self.robot=robot
        obs=unary_union(field.obstacles)
        self.free=polygonal(unary_union([field.field,*field.corridors]).difference(obs))
        self.original_task=polygonal(field.field.difference(obs))
        self.target=polygonal(self.original_task.buffer(-manual_setback,quad_segs=32))
        self.manual_predeclared=self.original_task.difference(self.target)
        # Extra reserve exceeds the polygonal round-join approximation error.
        self.cspace=polygonal(self.free.buffer(-robot.planning_radius,quad_segs=32))
        self.bounds=self.free.bounds
        self._prep_free=prep(self.free);self._prep_cs=prep(self.cspace)
        self._points={};self._segments={};self.exact_straight_acceptances=0
        if self.free.is_empty:raise PlanningError('No physical free space')

    def point_free(self,p):
        p=tuple(map(float,p))
        if p not in self._points:self._points[p]=bool(self._prep_cs.covers(Point(p)))
        return self._points[p]

    def segment_free(self,a,b):
        a,b=tuple(map(float,a)),tuple(map(float,b));key=(a,b) if a<=b else (b,a)
        if key in self._segments:return self._segments[key]
        okay=False
        if self.point_free(a) and self.point_free(b):
            if math.dist(a,b)<1e-10:okay=True
            elif self._prep_cs.covers(LineString([a,b])):okay=True
            else:
                g=translation_sweep(a,b,self.robot,self.robot.clearance_m+self.robot.geometry_reserve_m)
                okay=bool(self._prep_free.covers(g))
                if okay:self.exact_straight_acceptances+=1
        # Caches must not grow without bound on repeated sampling experiments.
        if len(self._segments)>200000:self._segments.clear()
        self._segments[key]=okay
        return okay

    def path_free(self,path):
        return bool(path) and all(self.point_free(p) for p in path) and all(self.segment_free(a,b) for a,b in zip(path,path[1:]))

    def sample(self,rng):
        x0,y0,x1,y1=self.bounds
        for _ in range(20000):
            p=(float(rng.uniform(x0,x1)),float(rng.uniform(y0,y1)))
            if self.point_free(p):return p
        raise PlanningError('Sampling failed: too little safe pivot space')

    def clearance_at(self,p):return Point(p).distance(self.free.boundary)


def save_geojson(path,features):
    data={'type':'FeatureCollection','coordinate_system':'local Cartesian metres',
          'features':[{'type':'Feature','properties':{'kind':name,**props},'geometry':mapping(g)}
                      for name,g,props in features]}
    Path(path).write_text(json.dumps(data,ensure_ascii=False,allow_nan=False),encoding='utf-8')
