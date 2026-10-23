"""Seeded synthetic metric fields. Robot size is never rescaled with the map."""
from __future__ import annotations
import math
import numpy as np
from shapely import affinity
from shapely.geometry import Polygon, Point, box, mapping
from .geometry import FieldMap

SHAPES=('rectangle','l_shape','trapezoid','notched','orchard','split')
DENSITIES=('none','low','medium','high')
DISTRIBUTIONS=('uniform','clustered','rows')

def generate(shape_name='rectangle',width=18.,height=14.,density='medium',distribution='uniform',seed=7):
    if shape_name not in SHAPES or density not in DENSITIES or distribution not in DISTRIBUTIONS:
        raise ValueError('Unknown map shape, density, or distribution')
    if not (math.isfinite(width) and math.isfinite(height) and 6<=width<=150 and 6<=height<=150):
        raise ValueError('Synthetic field dimensions must be 6..150 metres')
    w,h=float(width),float(height)
    if shape_name in ('rectangle','orchard','split'):field=box(0,0,w,h)
    elif shape_name=='l_shape':field=Polygon([(0,0),(w,0),(w,.48*h),(.58*w,.48*h),(.58*w,h),(0,h)])
    elif shape_name=='trapezoid':field=Polygon([(0,0),(w,0),(.82*w,h),(.12*w,h)])
    else:field=Polygon([(0,0),(w,0),(w,h),(.67*w,h),(.67*w,.7*h),(.47*w,.7*h),(.47*w,h),(0,h)])
    rng=np.random.default_rng(seed);obstacles=[]
    if shape_name=='split':obstacles.append(box(.47*w,-.1,.53*w,h+.1))
    count=0 if density=='none' else max(1,int(field.area/{'low':150,'medium':75,'high':40}[density]))
    count=min(count,150)
    accepted=0;candidate_count=0
    for i in range(count*250+1):
        if accepted>=count:break
        candidate_count+=1
        if distribution=='rows' or shape_name=='orchard':
            nx=max(1,int((w-4)/4));ny=max(1,int((h-4)/4));index=i%(nx*ny)
            x=2.5+(w-5)*(index%nx)/max(1,nx-1);y=2.5+(h-5)*(index//nx)/max(1,ny-1)
            radius=.50
        elif distribution=='clustered':
            x=float(rng.normal(.60*w,.18*w));y=float(rng.normal(.56*h,.20*h));radius=float(rng.uniform(.45,.9))
        else:x=float(rng.uniform(1.8,w-1.8));y=float(rng.uniform(1.8,h-1.8));radius=float(rng.uniform(.45,1.05))
        if accepted%3==1 and shape_name!='orchard':
            g=affinity.rotate(box(x-radius,y-.7*radius,x+radius,y+.7*radius),float(rng.uniform(0,150)))
        else:g=Point(x,y).buffer(radius,quad_segs=24)
        # Avoid manufacturing centimetre cracks in the regular benchmark set.
        if not field.buffer(-1.3).covers(g):continue
        if any(g.distance(o)<1.85 for o in obstacles):continue
        obstacles.append(g);accepted+=1
    fs=[{'type':'Feature','properties':{'kind':'field'},'geometry':mapping(field)}]
    fs += [{'type':'Feature','properties':{'kind':'obstacle','id':i},'geometry':mapping(g)} for i,g in enumerate(obstacles)]
    data={'type':'FeatureCollection','coordinate_system':'local Cartesian metres',
          'metadata':{'synthetic':True,'shape':shape_name,'width_m':w,'height_m':h,'density_request':density,
                      'distribution':distribution,'seed':seed,'requested_obstacles':count,
                      'placed_obstacles':len(obstacles),'actual_obstacle_area_fraction':sum(o.intersection(field).area for o in obstacles)/field.area,
                      'note':'minimum separation applied; requested count may not fit; split includes a deliberate impassable wall'},
          'features':fs}
    return FieldMap.from_geojson(data)
