"""Turn a certified polyline into certified motion geometry without changing order.

No unconstrained RDP: deleted coverage corners must remain under the cutter.
For positive-radius vehicles, a failed local blend may use a checked Dubins
loop (recorded), but NEVER an in-place turn or a smaller radius. This is a
bounded local converter, not a complete SE(2) obstacle planner.
"""
from __future__ import annotations
from functools import lru_cache
import math
import numpy as np
from scipy.optimize import minimize
from shapely.geometry import LineString
from .models import PlanningError
from .motion import merge_collinear
from .curves import (line,arc,bezier,at,endpoints,curvature,bounds,flatten,
                     certify_collision,wrap,length)
from .dubins import candidates


def _heading(leg):
    a,b=leg['p0'],leg['p1']
    return math.atan2(b[1]-a[1],b[0]-a[0])


def _controls(p,u,v,d,h1=.30,h2=.30):
    a=p-d*u;b=p+d*v
    return np.array([a,a+h1*d*u,a+2*h1*d*u,b-2*h2*d*v,b-h2*d*v,b])


@lru_cache(maxsize=512)
def _optimized_shape(angle,kappa_limit,weight):
    """Dimensionless two-handle SLSQP with actual numerical curvature constraints.

    This is candidate generation only. Bernstein bounds, swept-body and service
    checks are always applied afterwards, including when SLSQP reports success.
    """
    u=np.array([1.,0.]);v=np.array([math.cos(angle),math.sin(angle)]);p=np.zeros(2)
    samples=np.linspace(0,1,81)
    def curve(x):return bezier(_controls(p,u,v,1.,*x))
    def objective(x):
        g=curve(x);_,vel,acc,_,_,_=at(g,samples)
        speed=np.linalg.norm(vel,axis=1);k=curvature(g,samples)
        # Dimensionless length + curvature-variation energy (not physical fuel).
        energy=np.trapezoid(np.gradient(k,samples)**2/np.maximum(speed,1e-9),samples)
        return float(np.trapezoid(speed,samples)+weight*.02*energy)
    def constraints(x):
        g=curve(x)
        return np.r_[kappa_limit*.96-np.abs(curvature(g,samples)),np.linalg.norm(at(g,samples)[1],axis=1)-.08]
    initial=np.array([.30,.30]);old=objective(initial)
    opt=minimize(objective,initial,method='SLSQP',bounds=[(.12,.42),(.12,.42)],
                 constraints=[{'type':'ineq','fun':constraints}],options={'maxiter':60,'ftol':1e-8})
    okay=bool(opt.success and min(constraints(opt.x))>=-1e-7 and objective(opt.x)<=old+1e-8)
    return tuple(map(float,opt.x if okay else initial)),{
        'requested':'bezier_slsqp','solver_success':bool(opt.success),'accepted_by_numerical_constraints':okay,
        'objective_before':old,'objective_after':objective(opt.x) if okay else old,
        'iterations':int(opt.nit),'message':str(opt.message)}


def _service_preserved(gs,source_points,robot,cfg):
    if cfg.curve_service_rule=='centre_exact':
        # General rounded corners change the centre trace; do not disguise this.
        return False
    points=[]
    for g in gs:
        _,xy=flatten(g,cfg.curve_tolerance_m,cfg.curve_step_m)
        points.extend(xy.tolist() if not points else xy[1:].tolist())
    if len(points)<2:return False
    inner=LineString(points).buffer(robot.cutter_radius-cfg.curve_tolerance_m-1e-5,quad_segs=24)
    return bool(inner.covers(LineString(source_points)))


def _safe_dubins(q0,q1,world,robot,cfg,check):
    pool=[]
    for f in (1.,1.2,1.5):
        pool.extend(candidates(q0,q1,cfg.min_turn_radius_m*f))
    for row in sorted(pool,key=lambda r:r['length']):
        check()
        if all(certify_collision(g,world,robot,cfg.curve_tolerance_m)[0] for g in row['curves']):
            return row
    return None


def _blend(p,prev,nxt,world,robot,cfg,check):
    y0,y1=_heading(prev),_heading(nxt);delta=wrap(y1-y0)
    if abs(delta)<1e-6 or abs(delta)>math.pi-.06:return None
    u=np.array([math.cos(y0),math.sin(y0)]);v=np.array([math.cos(y1),math.sin(y1)])
    p=np.array(p,float)
    max_d=min(.44*math.dist(prev['p0'],prev['p1']),.44*math.dist(nxt['p0'],nxt['p1']),cfg.blend_distance_m)
    if max_d<1e-5:return None
    is_work=prev['kind']=='WORK' or nxt['kind']=='WORK'
    if cfg.trajectory_method=='circular_fillet':
        wanted=cfg.min_turn_radius_m*math.tan(abs(delta)/2)
        if wanted>max_d+1e-8:return None
        ds=[wanted]
    else:
        ds=sorted({min(max_d,f*cfg.min_turn_radius_m) for f in (1.25,1.5,2.,3.)} | {max_d},reverse=True)
    for d in ds:
        check()
        proposals=[]
        if cfg.trajectory_method=='circular_fillet':
            r=d/math.tan(abs(delta)/2);sign=1 if delta>0 else -1
            a=p-d*u;c=a+sign*r*np.array([-u[1],u[0]])
            proposals.append((arc(c,r,y0-sign*math.pi/2,delta),{}))
        else:
            if cfg.trajectory_method=='bezier_slsqp':
                h,record=_optimized_shape(float(delta),float(d/cfg.min_turn_radius_m),float(cfg.curve_shape_weight))
                proposals.append((bezier(_controls(p,u,v,d,*h)),record))
                # A numerically optimized candidate may fail the independent
                # continuous certificate. Try the original G2 shape BEFORE
                # spending a larger forward loop; never relax radius/collision.
                if h != (.30,.30):
                    proposals.append((bezier(_controls(p,u,v,d)),{
                        **record,'accepted_by_numerical_constraints':False,
                        'shape_fallback':'fixed_g2_after_optimized_candidate_not_certified',
                        'objective_after':record['objective_before']}))
            else:proposals.append((bezier(_controls(p,u,v,d)),{}))
        for g,record in proposals:
            a,b,_,_=endpoints(g)
            try:
                bd=bounds(g)
                if bd['curvature']>1/cfg.min_turn_radius_m+1e-7:continue
                if not certify_collision(g,world,robot,cfg.curve_tolerance_m)[0]:continue
                if is_work and not _service_preserved([g],[a,p,b],robot,cfg):continue
            except ValueError:
                continue
            return {'curve':g,'in':a.tolist(),'out':b.tolist(),'kind':'WORK' if is_work else 'TRANSIT',
                    'record':record,'certified_curvature_bound':bd['curvature']}

    return None


def build_geometry(start,legs,world,robot,cfg,check=lambda:None):
    # Every collinear merge preserves all original task centres and tool states.
    raw=merge_collinear(legs)
    records=[];primitives=[];blends={};bridges={};skip=set()
    if not raw:
        return [],{'requested_method':cfg.trajectory_method,'records':[],'curve_counts':{},'pivot_fallbacks':0}
    # Replace a short cross-row bridge by a pose-constrained forward U-turn.
    # The two long work rows remain unchanged. This works also for close rows
    # (<2R): RLR/LRL loops are considered, rather than inventing a smaller R.
    for j in range(1,len(raw)-1):
        check()
        if j-1 in bridges or j+1 in bridges:continue
        a,b,c=raw[j-1:j+2];ya,yb,yc=map(_heading,(a,b,c))
        da,db,dc=(math.dist(x['p0'],x['p1']) for x in (a,b,c))
        u_turn=(abs(abs(wrap(yc-ya))-math.pi)<1e-5 and abs(abs(wrap(yb-ya))-math.pi/2)<1e-5
                and db<max(2.8*cfg.min_turn_radius_m,.8) and min(da,dc)>max(1.,2*db))
        if not u_turn:continue
        row=_safe_dubins((*b['p0'],ya),(*b['p1'],yc),world,robot,cfg,check)
        if row is None:continue
        work=b['kind']=='WORK'
        if work and cfg.curve_service_rule=='cutter_footprint' and not _service_preserved(row['curves'],[b['p0'],b['p1']],robot,cfg):continue
        if work and cfg.curve_service_rule=='centre_exact' and b.get('tasks'):
            # Only accept this bridge when all its explicit tasks are its endpoint.
            # The global independent exact-centre check is authoritative.
            pass
        bridges[j]=row;skip.update((j,j+1))
        records.append({'segment':j,'actual':'dubins_cross_row','word':row['word'],'radius_m':row['radius']})
    if cfg.trajectory_method!='dubins':
        for i in range(1,len(raw)):
            check()
            if i in skip:continue
            blend=_blend(raw[i]['p0'],raw[i-1],raw[i],world,robot,cfg,check)
            if blend is not None:
                blends[i]=blend
                records.append({'corner':i,'actual':cfg.trajectory_method,'curvature_bound':blend['certified_curvature_bound'],**blend['record']})
    current=np.asarray(start,float);yaw=_heading(raw[0]);pivots=0
    initial_yaw=yaw  # v2 explicitly initializes the robot tangent to the first run.
    def append_curve(g,kind,cluster,reason,turn_id=None):
        nonlocal current,yaw
        a,b,ya,yb=endpoints(g)
        if math.dist(a,current)>2e-6:raise PlanningError('Curve assembly position discontinuity')
        if abs(wrap(ya-yaw))>2e-5:raise PlanningError('Curve assembly tangent discontinuity')
        primitives.append({'geometry':g,'kind':kind,'cluster':cluster,'reason':reason,'turn_id':turn_id})
        current=b;yaw=yaw+wrap(yb-yaw) if g['type']!='ARC' else yaw+g['sweep']
        if len(primitives)>cfg.max_curve_primitives:raise PlanningError('Curve primitive budget exceeded')
    for j,leg in enumerate(raw):
        check()
        if j in bridges:
            row=bridges[j]
            for g in row['curves']:append_curve(g,leg['kind'],leg['cluster'],'certified_dubins_cross_row',f'bridge_{j}')
            continue
        a=np.asarray(blends[j]['out'] if j in blends else leg['p0'],float)
        b=np.asarray(blends[j+1]['in'] if j+1 in blends else leg['p1'],float)
        target_yaw=_heading(leg)
        if math.dist(current,a)>2e-6:raise PlanningError('Trimmed route disconnected')
        delta=wrap(target_yaw-yaw)
        if abs(delta)>1e-6:
            row=None
            if (cfg.motion_model=='bounded_curvature' or cfg.trajectory_method=='dubins'
                    or cfg.turn_policy in ('loops_only','loop_then_pivot')):
                row=_safe_dubins((*current,yaw),(*current,target_yaw),world,robot,cfg,check)
            if row is not None:
                before_yaw=float(yaw);turn_position=current.tolist()
                for g in row['curves']:append_curve(g,'TRANSIT',leg['cluster'],'certified_dubins_heading_loop',f'heading_{j}')
                records.append({'corner':j,'actual':'dubins_heading_loop','word':row['word'],'radius_m':row['radius'],
                    'turn_id':f'heading_{j}','position':turn_position,'yaw_before':before_yaw,
                    'yaw_after':float(yaw),'delta_heading_rad':float(delta),'length_m':row['length']})
            elif (cfg.motion_model=='differential' and cfg.curve_allow_pivot_fallback
                    and cfg.turn_policy!='loops_only'):
                primitives.append({'kind':'TURN','p':current.tolist(),'yaw0':yaw,'yaw1':yaw+delta,
                                   'cluster':leg['cluster'],'reason':'reported_differential_pivot_fallback'})
                yaw+=delta;pivots+=1
                records.append({'corner':j,'actual':'pivot_fallback','reason':'no_certified_radius_blend_or_loop'})
            else:
                raise PlanningError(f'No safe {cfg.min_turn_radius_m:.3f} m-radius turn at corner {j}, '
                    f'position {np.round(current,3).tolist()}. No pivot or smaller-radius substitution was made. '
                    'Use a sweep-friendly order / a larger predeclared setback / a different valid radius. '
                    'This local converter is not a complete SE(2) planner.')
        if math.dist(a,b)>1e-8:append_curve(line(a,b),leg['kind'],leg['cluster'],'retained_certified_straight')
        if j+1 in blends:
            z=blends[j+1];append_curve(z['curve'],z['kind'],leg['cluster'],'certified_coverage_preserving_blend')
    from collections import Counter
    counts=Counter(p['geometry']['type'] for p in primitives if 'geometry' in p)
    return primitives,{'requested_method':cfg.trajectory_method,'motion_model':cfg.motion_model,
        'turn_policy':cfg.turn_policy,
        'heading_loops':sum(r.get('actual')=='dubins_heading_loop' for r in records),
        'cross_row_loops':sum(r.get('actual')=='dubins_cross_row' for r in records),
        'initial_yaw_rad':initial_yaw,'initial_heading_policy':'tangent of first route leg; not a heading reset during motion',
        'raw_length_m':sum(math.dist(l['p0'],l['p1']) for l in legs),
        'curve_counts':dict(counts),'pivot_fallbacks':pivots,'records':records,
        'scope':'local curve conversion of fixed coverage order; not globally optimal or SE(2)-complete',
        'service_rule':cfg.curve_service_rule,'curvature_bound_inv_m':1/cfg.min_turn_radius_m}
