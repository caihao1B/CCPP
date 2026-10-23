"""Analytical LINE, ARC and quintic Bezier geometry (u in [0, 1]).

The animation evaluates these curves, NOT straight interpolation of waypoints.
Collision checks bound the complete curve by sub-control-polygon convex hulls
or circular-arc chord+sagitta enclosures. A circumscribed body disk is conservative
for curved motion; straight translations retain the original rectangle test.
"""
from __future__ import annotations
from functools import lru_cache
import math
import numpy as np
from scipy.integrate import quad
from shapely.geometry import Point, LineString, MultiPoint
from .geometry import translation_sweep


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _bernstein(control, u):
    c = np.asarray(control, float)
    u = np.asarray(u, float).reshape(-1)
    n = len(c) - 1
    if n < 0:
        return np.zeros((len(u), 2))
    basis = np.array([math.comb(n, k) * (1-u)**(n-k) * u**k for k in range(n+1)]).T
    return basis @ c


def split_control(control):
    levels = [np.asarray(control, float)]
    while len(levels[-1]) > 1:
        levels.append((levels[-1][:-1] + levels[-1][1:]) / 2)
    return np.array([a[0] for a in levels]), np.array([a[-1] for a in reversed(levels)])


def derivatives(control, order=4):
    arrays = [np.asarray(control, float)]
    for _ in range(order):
        old = arrays[-1]
        arrays.append((len(old)-1)*np.diff(old, axis=0) if len(old)>1 else np.zeros((1,2)))
    return arrays


def line(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    if np.linalg.norm(b-a) < 1e-10:
        raise ValueError('A moving line must have positive length')
    return {'type': 'LINE', 'p0': a.tolist(), 'p1': b.tolist()}


def arc(center, radius, angle0, sweep):
    if radius <= 0 or abs(sweep) < 1e-10:
        raise ValueError('A circular arc needs positive radius and nonzero angle')
    return {'type': 'ARC', 'center': list(map(float, center)), 'radius': float(radius),
            'angle0': float(angle0), 'sweep': float(sweep)}


def bezier(control):
    a = np.asarray(control, float)
    if a.shape != (6, 2) or not np.all(np.isfinite(a)):
        raise ValueError('BEZIER5 requires six finite 2D control points')
    return {'type': 'BEZIER5', 'control': a.tolist()}


def validate_descriptor(g):
    typ=g.get('type')
    if typ=='LINE':
        a=np.asarray([g['p0'],g['p1']],float)
        if a.shape!=(2,2) or not np.all(np.isfinite(a)) or np.linalg.norm(a[1]-a[0])<1e-10:
            raise ValueError('Invalid LINE descriptor')
    elif typ=='ARC':
        c=np.asarray(g['center'],float)
        r=float(g['radius']);theta=float(g['sweep']);angle=float(g['angle0'])
        if c.shape!=(2,) or not np.all(np.isfinite(c)) or not all(math.isfinite(x) for x in (r,theta,angle)) or r<=0 or not 1e-10<=abs(theta)<=2*math.pi+1e-6:
            raise ValueError('Invalid ARC radius/sweep/center')
    elif typ=='BEZIER5':
        a=np.asarray(g['control'],float)
        if a.shape!=(6,2) or not np.all(np.isfinite(a)):
            raise ValueError('Invalid BEZIER5 descriptor')
    else:raise ValueError('Unknown curve descriptor type')
    return g


def at(g, u):
    """Return position and its first four derivatives w.r.t. u, plus yaw."""
    validate_descriptor(g)
    u = np.clip(np.asarray(u, float).reshape(-1), 0, 1)
    typ = g['type']
    if typ == 'LINE':
        a, b = np.asarray(g['p0'], float), np.asarray(g['p1'], float)
        d = b-a
        vals = [a+u[:,None]*d, np.broadcast_to(d, (len(u),2)).copy()]
        vals += [np.zeros((len(u),2)) for _ in range(3)]
        yaw = np.full(len(u), math.atan2(d[1],d[0]))
    elif typ == 'ARC':
        r, theta = float(g['radius']), float(g['sweep'])
        a = float(g['angle0']) + theta*u
        vals = [np.asarray(g['center']) + r*np.column_stack([np.cos(a),np.sin(a)])]
        for k in range(1,5):
            vals.append(r*theta**k*np.column_stack([np.cos(a+k*math.pi/2),np.sin(a+k*math.pi/2)]))
        yaw = a + math.copysign(math.pi/2, theta)
    elif typ == 'BEZIER5':
        vals = [_bernstein(c,u) for c in derivatives(g['control'])]
        yaw = np.arctan2(vals[1][:,1],vals[1][:,0])
    else:
        raise ValueError(f'Unknown curve type {typ!r}')
    return (*vals, yaw)


def endpoints(g):
    *vals, yaw = at(g,[0,1])
    p = vals[0]
    return p[0],p[1],float(yaw[0]),float(yaw[1])


def curvature(g, u):
    _,d,a,_,_,_ = at(g,u)
    q = np.sum(d*d,axis=1)
    if np.any(q <= 1e-20):
        raise ValueError('Curve contains a stationary parameter/cusp')
    return (d[:,0]*a[:,1]-d[:,1]*a[:,0]) / q**1.5


def length(g):
    if g['type']=='LINE':
        return math.dist(g['p0'],g['p1'])
    if g['type']=='ARC':
        return g['radius']*abs(g['sweep'])
    return float(quad(lambda u: float(np.linalg.norm(at(g,[u])[1][0])),0,1,epsabs=1e-9,limit=80)[0])


def _point_segment_distance(points,a,b):
    points=np.asarray(points,float);a=np.asarray(a,float);d=np.asarray(b,float)-a
    if float(d@d)<1e-24:return np.linalg.norm(points-a,axis=1)
    t=np.clip(((points-a)@d)/(d@d),0,1)
    return np.linalg.norm(points-a-t[:,None]*d,axis=1)


def flatten(g, tolerance=.002, max_step=.06):
    """Return u,xy with a continuous Hausdorff error <= tolerance.

    Bezier sub-control polygons lie in the endpoint-segment tolerance tube.
    Intermediate-value continuity supplies the reverse Hausdorff inclusion.
    ARC uses its exact sagitta. This is NOT the collision checker.
    """
    if not 0<tolerance<=.05 or max_step<=0:
        raise ValueError('Invalid curve flattening tolerance/step')
    if g['type']=='LINE':
        u=np.linspace(0,1,max(2,math.ceil(length(g)/max_step)+1))
        return u,at(g,u)[0]
    if g['type']=='ARC':
        r=g['radius'];angle=min(.25,2*math.acos(max(-1.,1-min(tolerance,r)/r)),max_step/r)
        n=max(1,math.ceil(abs(g['sweep'])/max(angle,1e-6)))
        if n>100000:raise ValueError('Arc sampling budget exceeded')
        u=np.linspace(0,1,n+1)
        return u,at(g,u)[0]
    out=[(0.,np.asarray(g['control'][0]))]
    stack=[(np.asarray(g['control']),0.,1.,0)]
    while stack:
        cp,lo,hi,depth=stack.pop()
        flat=float(max(_point_segment_distance(cp,cp[0],cp[-1])))
        span=float(np.linalg.norm(np.ptp(cp,axis=0)))
        if flat<=tolerance and span<=max_step:
            out.append((hi,cp[-1]));continue
        if depth>=20:raise ValueError('Bezier flattening budget exceeded')
        a,b=split_control(cp);mid=(lo+hi)/2
        stack.append((b,mid,hi,depth+1));stack.append((a,lo,mid,depth+1))
    return np.array([a for a,_ in out]),np.array([p for _,p in out])


def _enclosure_safe(world, hull, extra_radius):
    # Testing the complete enclosure and its distance to ALL boundaries also
    # considers holes. Distance alone would not reject a hull inside an obstacle.
    return bool(world.free.covers(hull) and hull.distance(world.free.boundary)>=extra_radius-1e-10)


def certify_collision(g,world,robot,tolerance=.002):
    """Conservative WHOLE-CURVE rectangular-body containment certificate.

    Curves use the circumscribed disk of the rectangle at every yaw. Failures
    may be conservative; never reinterpret one as a proof of unreachability.
    """
    validate_descriptor(g)
    margin=robot.radius+robot.clearance_m+robot.geometry_reserve_m
    if g['type']=='LINE':
        padded=translation_sweep(g['p0'],g['p1'],robot,robot.clearance_m+robot.geometry_reserve_m)
        return bool(world.free.covers(padded)),1
    if g['type']=='ARC':
        r=g['radius'];theta=g['sweep']
        step=min(.20,2*math.acos(max(-1.,1-min(tolerance,r)/r)))
        n=max(1,math.ceil(abs(theta)/max(step,1e-6)))
        if n>30000:return False,n
        xy=at(g,np.linspace(0,1,n+1))[0]
        sag=r*(1-math.cos(abs(theta)/(2*n)))
        chord=LineString(xy)
        return _enclosure_safe(world,chord,margin+sag),n
    stack=[(np.asarray(g['control']),0)];count=0
    while stack:
        cp,depth=stack.pop();count+=1
        hull=MultiPoint(cp).convex_hull
        if _enclosure_safe(world,hull,margin):continue
        if depth>=16 or np.linalg.norm(np.ptp(cp,axis=0))<tolerance:
            return False,count
        if count>30000:return False,count
        a,b=split_control(cp);stack.extend([(a,depth+1),(b,depth+1)])
    return True,count


def _cross_bound(a,b):
    return float(np.max(np.abs(a[:,None,0]*b[None,:,1]-a[:,None,1]*b[None,:,0])))


def _dot_bound(a,b):
    return float(np.max(np.abs(a@b.T)))


@lru_cache(maxsize=4096)
def _bezier_bounds_cached(relative_control, depth):
    cp=np.array(relative_control).reshape(6,2)
    stack=[(cp,1.,0)];bound={k:0. for k in
        ('speed_u','speed_du','speed_ddu','yaw_u','yaw_uu','yaw_uuu','curvature','curvature_u')}
    minimum=math.inf
    while stack:
        p,width,level=stack.pop()
        if level<depth:
            a,b=split_control(p);stack.extend([(a,width/2,level+1),(b,width/2,level+1)]);continue
        dd=derivatives(p)
        v,a,b,c=[dd[k]/width**k for k in range(1,5)]
        m=float(MultiPoint(v).convex_hull.distance(Point(0,0)))
        if m<1e-10:
            raise ValueError('Cannot certify a nonzero tangent (possible cusp)')
        m1=float(np.max(np.linalg.norm(v,axis=1)));m2=float(np.max(np.linalg.norm(a,axis=1)))
        m3=float(np.max(np.linalg.norm(b,axis=1)))
        n=_cross_bound(v,a);np1=_cross_bound(v,b);npp=_cross_bound(a,b)+_cross_bound(v,c)
        dot=_dot_bound(v,a);qp=2*dot;qpp=2*(m2*m2+_dot_bound(v,b));q=m*m
        vals={'speed_u':m1,'speed_du':dot/m,
              'speed_ddu':(m2*m2+_dot_bound(v,b))/m+dot*dot/m**3,
              'yaw_u':n/q,'yaw_uu':np1/q+n*qp/q**2,
              'yaw_uuu':npp/q+2*np1*qp/q**2+n*qpp/q**2+2*n*qp**2/q**3,
              'curvature':n/m**3,'curvature_u':np1/m**3+3*n*dot/m**5}
        for k,x in vals.items():bound[k]=max(bound[k],x)
        minimum=min(minimum,m)
    # Tiny numeric reserve in addition to profile/geometry reserves.
    bound={k:float(v)*(1+2e-7)+1e-12 for k,v in bound.items()}
    bound['minimum_parameter_speed']=minimum
    bound['bound_method']='subdivided Bernstein derivative convex-hull bounds'
    bound['subdivision_depth']=depth
    return bound


def bounds(g,depth=4):
    validate_descriptor(g)
    if g['type']=='BEZIER5':
        a=np.asarray(g['control']);a=a-a[0]
        # No rounding: certificate caches do not change geometry.
        return dict(_bezier_bounds_cached(tuple(a.ravel().tolist()),depth))
    L=length(g);theta=abs(g['sweep']) if g['type']=='ARC' else 0.
    return {'speed_u':L,'speed_du':0.,'speed_ddu':0.,'yaw_u':theta,'yaw_uu':0.,'yaw_uuu':0.,
            'curvature':1/g['radius'] if g['type']=='ARC' else 0.,'curvature_u':0.,
            'minimum_parameter_speed':L,'bound_method':'analytic line/constant-curvature arc'}
