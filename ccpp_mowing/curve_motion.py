"""Finite-rate motion on ANALYTICAL curves; no waypoint teleportation.

Geometry parameter u(t) follows the existing quintic velocity-ramp Profile.
A two-variable SLSQP optimizes total duration and ramp fraction. Conservative
continuous derivative bounds enforce forward speed, longitudinal acceleration /
jerk, yaw derivatives, differential wheel limits, lateral acceleration and
(equivalent bicycle) steering rate. Every candidate is checked independently.

All primitives start/end at rest. At curvature jumps, bounded-curvature vehicles
have explicit standstill STEER events; they do not rotate their chassis in place.
This is deliberately conservative rather than claiming globally time-optimal
nonstop motion or hardware controller validation.
"""
from __future__ import annotations
from dataclasses import asdict
import math
import numpy as np
from scipy.optimize import minimize
from .models import PlanningError
from .motion import Profile,TimeOptimizer,KJ
from .curves import at,bounds,endpoints,curvature,wrap
from .curve_planner import build_geometry


def limits(robot,cfg,kind):
    return {'v':min(robot.work_speed_mps,cfg.curve_work_speed_mps) if kind=='WORK' else min(robot.transit_speed_mps,cfg.curve_transit_speed_mps),
        'a':min(robot.linear_acceleration_mps2,cfg.curve_acceleration_mps2),
        'j':min(robot.linear_jerk_mps3,cfg.curve_jerk_mps3),'w':robot.yaw_rate_radps,
        'alpha':robot.yaw_acceleration_radps2,'yaw_jerk':robot.yaw_jerk_radps3,
        'track_v':robot.track_speed_mps,'track_a':robot.track_acceleration_mps2,
        'track_j':robot.track_jerk_mps3,'lateral_a':cfg.lateral_acceleration_mps2,
        'steering_rate':cfg.steering_rate_radps}


def physical_bounds(bd,pr,robot,cfg):
    p,a,j=pr.peak,pr.max_acc,pr.max_jerk
    v=bd['speed_u']*p
    acc=bd['speed_du']*p*p+bd['speed_u']*a
    jerk=bd['speed_ddu']*p**3+3*bd['speed_du']*p*a+bd['speed_u']*j
    w=bd['yaw_u']*p;alpha=bd['yaw_uu']*p*p+bd['yaw_u']*a
    wj=bd['yaw_uuu']*p**3+3*bd['yaw_uu']*p*a+bd['yaw_u']*j
    half=robot.track_spacing_m/2
    return {'v':v,'a':acc,'j':jerk,'w':w,'alpha':alpha,'yaw_jerk':wj,
            'track_v':v+half*w,'track_a':acc+half*alpha,'track_j':jerk+half*wj,
            'lateral_a':bd['curvature']*v*v,
            'steering_rate':cfg.wheelbase_m*bd['curvature_u']*p}


def constraint_ratios(bd,pr,robot,cfg,kind):
    lim=limits(robot,cfg,kind);actual=physical_bounds(bd,pr,robot,cfg)
    keys=list(lim)
    if cfg.motion_model!='bounded_curvature':keys.remove('steering_rate')
    # Differential virtual-wheel bounds also conservatively limit the car-like
    # model; the GUI labels them as equivalent wheel speeds in that mode.
    return {k:actual[k]/lim[k] for k in keys}


def _profile(duration,fraction):
    return Profile(1.,1/((1-fraction)*duration),fraction*duration,(1-2*fraction)*duration)


def _feasible_duration(bd,fraction,robot,cfg,kind):
    pr=_profile(1.,fraction);ratios=constraint_ratios(bd,pr,robot,cfg,kind)
    degree={'v':1,'a':2,'j':3,'w':1,'alpha':2,'yaw_jerk':3,'track_v':1,'track_a':2,'track_j':3,'lateral_a':2,'steering_rate':1}
    return max(1e-4,max(v**(1/degree[k]) for k,v in ratios.items()))


class CurveTimeOptimizer:
    def __init__(self,robot,cfg):
        self.robot=robot;self.cfg=cfg;self.records=[];self.cache={};self.cache_hits=0

    def solve(self,g,kind):
        bd=bounds(g);cfg=self.cfg;robot=self.robot
        numeric=tuple((k,float(v)) for k,v in sorted(bd.items()) if isinstance(v,(int,float)))
        key=(numeric,kind,cfg.timing)
        if key in self.cache:
            self.cache_hits+=1
            pr,baseline=self.cache[key]
            return Profile(**asdict(pr)),baseline,bd
        fraction=.40
        baseline=_profile(_feasible_duration(bd,fraction,robot,cfg,kind)*1.25,fraction)
        accepted=False;pr=baseline
        rec={'method':cfg.timing,'curve_type':g['type'],'kind':kind,'baseline_seconds':baseline.duration}
        if cfg.timing=='slsqp':
            def con(x):
                p=_profile(float(x[0]),float(x[1]))
                return 1.-np.array(list(constraint_ratios(bd,p,robot,cfg,kind).values()))
            opt=minimize(lambda x:x[0],[baseline.duration,fraction],method='SLSQP',
                         bounds=[(1e-4,baseline.duration*2),(.08,.5)],
                         constraints=[{'type':'ineq','fun':con}],options={'maxiter':90,'ftol':1e-9})
            if opt.success and min(con(opt.x))>=-1e-7:
                candidate=_profile(float(opt.x[0]),float(opt.x[1])).scaled(1.005)
                if max(constraint_ratios(bd,candidate,robot,cfg,kind).values())<=1+1e-9 and candidate.duration<=baseline.duration:
                    pr=candidate;accepted=True
            rec.update(solver_success=bool(opt.success),iterations=int(opt.nit),message=str(opt.message))
            if not accepted:rec['fallback']='independently_validated_conservative_profile'
        rec.update(accepted=accepted,actual_seconds=pr.duration,
                   continuous_bound_ratios=constraint_ratios(bd,pr,robot,cfg,kind))
        self.records.append(rec);self.cache[key]=(pr,baseline.duration)
        return pr,baseline.duration,bd


def build_curve_events(start,legs,world,robot,cfg,check=lambda:None):
    primitives,geometry_report=build_geometry(start,legs,world,robot,cfg,check)
    opt=CurveTimeOptimizer(robot,cfg);simple=TimeOptimizer(cfg.timing)
    events=[];t=0.;p=list(start);yaw=float(geometry_report.get('initial_yaw_rad',0.));tool=False;steer=0.;baseline=0.
    def add(kind,duration,**extra):
        nonlocal t,p,yaw,steer,baseline
        if duration<1e-10:return
        e={'id':len(events),'kind':kind,'t0':t,'t1':t+duration,'duration':duration,
           'p0':list(p),'p1':list(p),'yaw0':yaw,'yaw1':yaw,'steer0':steer,'steer1':steer,
           'tool_on':False,'trajectory_v2':True,**extra}
        events.append(e);t+=duration;p=list(e['p1']);yaw=float(e['yaw1']);steer=float(e['steer1'])
        baseline+=e.get('baseline_seconds',duration)
        if len(events)>cfg.max_events:raise PlanningError('Trajectory event budget exceeded')
    def toggle(on):
        nonlocal tool
        if tool==on:return
        tool=on
        add('WAIT',robot.tool_spinup_s if on else robot.tool_spindown_s,
            reason='assumed_tool_spinup' if on else 'assumed_tool_spindown')
    toggle(True);add('SERVICE',1.,tool_on=True,reason='initial_stationary_service_assumed_1s',cluster=-1)
    for item in primitives:
        check()
        if item['kind']=='TURN':
            if cfg.motion_model=='bounded_curvature' or cfg.turn_policy=='loops_only':raise PlanningError('Internal error: forbidden pivot')
            toggle(False);d=abs(item['yaw1']-item['yaw0']);b=robot.track_spacing_m
            pr,base=simple.solve(d,min(robot.yaw_rate_radps,2*robot.track_speed_mps/b),
                                min(robot.yaw_acceleration_radps2,2*robot.track_acceleration_mps2/b),
                                min(robot.yaw_jerk_radps3,2*robot.track_jerk_mps3/b))
            add('TURN',pr.duration,yaw1=yaw+wrap(item['yaw1']-yaw),profile=asdict(pr),baseline_seconds=base,
                reason=item['reason'],cluster=item['cluster'])
            continue
        g=item['geometry'];a,b,ya,yb=endpoints(g)
        if math.dist(p,a)>2e-6 or abs(wrap(ya-yaw))>2e-5:raise PlanningError('Discontinuous curve event sequence')
        k0,k1=map(float,curvature(g,[0,1]));target=math.atan(cfg.wheelbase_m*k0)
        end_steer=math.atan(cfg.wheelbase_m*k1)
        if cfg.motion_model=='bounded_curvature' and abs(target-steer)>1e-6:
            toggle(False)
            # Finite wheel-steering actuation at zero chassis speed; not a pivot.
            delta=abs(target-steer)
            pr,base=simple.solve(delta,cfg.steering_rate_radps,.8,1.6)
            add('STEER',pr.duration,steer1=target,steer_profile=asdict(pr),baseline_seconds=base,
                reason='standstill_steering_at_curvature_jump',cluster=item['cluster'])
        kind=item['kind'];toggle(kind=='WORK')
        pr,base,bd=opt.solve(g,kind)
        dy=float(g['sweep']) if g['type']=='ARC' else wrap(yb-ya)
        add(kind,pr.duration,p1=b.tolist(),yaw1=yaw+dy,steer0=target,steer1=end_steer,
            tool_on=kind=='WORK',profile=asdict(pr),parameter='u',curve=g,
            curve_bounds=bd,baseline_seconds=base,reason=item['reason'],cluster=item['cluster'],turn_id=item.get('turn_id'))
    toggle(False)
    geometry_report.update(actual_duration_s=t,baseline_duration_s=baseline,
        time_reduction_ratio=1-t/baseline if baseline else 0.,
        profile_solves=len(opt.records),cache_hits=opt.cache_hits,
        accepted_numeric_solves=sum(r['accepted'] for r in opt.records),
        fallback_profile_count=sum('fallback' in r for r in opt.records),
        time_records=opt.records,mode=cfg.timing,
        timing_scope='independent rest-to-rest primitive timing; stops at all primitive joins, finite standstill steering at curvature jumps')
    return events,geometry_report


def evaluate_curve_event(e,t,robot):
    """Analytical pose + derivatives for one group of event-local times."""
    pr=Profile(**e['profile']);u,ud,udd,uddd=pr.at(t)
    pos,r1,r2,r3,r4,yaw=at(e['curve'],u)
    _,_,g_yaw0,_=endpoints(e['curve'])
    if e['curve']['type']=='BEZIER5':
        yaw=e['yaw0']+np.array([wrap(a-g_yaw0) for a in yaw])
    else:yaw=yaw+(e['yaw0']-g_yaw0)
    q=np.sum(r1*r1,axis=1);s=np.sqrt(q)
    cross=lambda a,b:a[:,0]*b[:,1]-a[:,1]*b[:,0]
    dot=np.sum(r1*r2,axis=1);n=cross(r1,r2);n1=cross(r1,r3)
    n2=cross(r2,r3)+cross(r1,r4);qp=2*dot;qpp=2*(np.sum(r2*r2,axis=1)+np.sum(r1*r3,axis=1))
    psi1=n/q;psi2=n1/q-n*qp/q**2
    psi3=n2/q-2*n1*qp/q**2-n*qpp/q**2+2*n*qp**2/q**3
    su=dot/s;suu=(np.sum(r2*r2,axis=1)+np.sum(r1*r3,axis=1))/s-dot*dot/s**3
    v=s*ud;a=su*ud*ud+s*udd;j=suu*ud**3+3*su*ud*udd+s*uddd
    w=psi1*ud;alpha=psi2*ud**2+psi1*udd
    wj=psi3*ud**3+3*psi2*ud*udd+psi1*uddd
    k=n/q**1.5;ku=n1/q**1.5-3*n*dot/q**2.5
    return {'pos':pos,'yaw':yaw,'v':v,'a':a,'j':j,'omega':w,'alpha':alpha,'yaw_jerk':wj,
            'kappa':k,'kappa_u':ku,'u_dot':ud,'lateral_a':k*v*v}


def curve_diagnostics(events,times,robot,cfg):
    """Same time/pose law as the GUI, with curvature and steering telemetry."""
    times=np.clip(np.asarray(times,float),0,events[-1]['t1'])
    index=np.minimum(np.searchsorted([e['t1'] for e in events],times,side='right'),len(events)-1)
    out=np.zeros((len(times),8));out[:,0]=times;out[:,7]=index
    for idx in np.unique(index):
        mask=index==idx;e=events[int(idx)];local=times[mask]-e['t0']
        if 'curve' in e:
            val=evaluate_curve_event(e,local,robot);k=val['kappa']
            out[mask,1]=k;out[mask,2]=np.arctan(cfg.wheelbase_m*k)
            out[mask,3]=cfg.wheelbase_m*val['kappa_u']*val['u_dot']/(1+(cfg.wheelbase_m*k)**2)
            out[mask,4]=val['lateral_a'];out[mask,5]=val['j'];out[mask,6]=val['yaw_jerk']
        elif 'steer_profile' in e:
            pr=Profile(**e['steer_profile']);d,v,_,_=pr.at(local);sgn=1 if e['steer1']>=e['steer0'] else -1
            out[mask,2]=e['steer0']+sgn*d;out[mask,3]=sgn*v
        else:out[mask,2]=e.get('steer0',0.)
    return out

DIAGNOSTIC_COLUMNS=['t','curvature_inv_m','equivalent_steering_rad','steering_rate_radps',
                    'lateral_acceleration_mps2','longitudinal_jerk_mps3','yaw_jerk_radps3','event_id']
