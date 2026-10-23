"""Finite-rate stop-turn-go execution + numerical time-profile optimization.

For each straight/rotate primitive choose peak speed V and ramp time r:
  minimize T=d/V+r
  s.t. V*r<=d, 1.875*V/r<=amax, (10/sqrt(3))*V/r²<=jmax,
       0<V<=vmax.
The velocity ramp is V*(10u³-15u⁴+6u⁵). Piecewise analytic bounds validate the
whole primitive, not just samples. This is time optimization on a fixed path,
NOT a global trajectory optimizer or a claim of vehicle dynamics fidelity.
"""
from __future__ import annotations
from dataclasses import dataclass, asdict
import bisect, math
import numpy as np
from scipy.optimize import minimize
from .models import PlanningError
from .geometry import dedup

KJ=10/math.sqrt(3)


def wrap(a):return (a+math.pi)%(2*math.pi)-math.pi

@dataclass
class Profile:
    distance: float
    peak: float
    ramp: float
    cruise: float

    @property
    def duration(self):return 2*self.ramp+self.cruise
    @property
    def max_acc(self):return 1.875*self.peak/self.ramp
    @property
    def max_jerk(self):return KJ*self.peak/self.ramp**2

    @classmethod
    def feasible(cls,d,vmax,amax,jmax):
        if not all(math.isfinite(x) and x>0 for x in (d,vmax,amax,jmax)):raise ValueError('Positive finite bounds required')
        v=.999*min(vmax,math.sqrt(d*amax/1.875),(d*d*jmax/KJ)**(1/3))
        r=max(1.875*v/amax,math.sqrt(KJ*v/jmax));return cls(d,v,r,max(0.,d/v-r))

    def scaled(self,factor):
        return Profile(self.distance,self.peak/factor,self.ramp*factor,self.cruise*factor)

    def at(self,t):
        t=np.clip(np.asarray(t,dtype=float),0,self.duration);r,c,V=self.ramp,self.cruise,self.peak
        def h(u):return 10*u**3-15*u**4+6*u**5
        def dh(u):return 30*u**2-60*u**3+30*u**4
        def ddh(u):return 60*u-180*u**2+120*u**3
        def integ(u):return 2.5*u**4-3*u**5+u**6
        u=np.clip(t/r,0,1);s=V*r*integ(u);v=V*h(u);a=V/r*dh(u);j=V/r**2*ddh(u)
        mid=(t>=r)&(t<r+c);s=np.where(mid,V*r/2+V*(t-r),s);v=np.where(mid,V,v)
        a=np.where(mid,0,a);j=np.where(mid,0,j)
        u=np.clip((t-r-c)/r,0,1);down=t>=r+c
        s=np.where(down,V*(r/2+c)+V*r*(u-integ(u)),s);v=np.where(down,V*(1-h(u)),v)
        a=np.where(down,-V/r*dh(u),a);j=np.where(down,-V/r**2*ddh(u),j)
        return s,v,a,j

class TimeOptimizer:
    def __init__(self,mode='slsqp'):
        self.mode=mode;self.cache={};self.records=[];self.cache_hits=0

    def solve(self,d,vmax,amax,jmax):
        d=float(d);key=(round(d,8),vmax,amax,jmax,self.mode)
        baseline=Profile.feasible(d,.65*vmax,.75*amax,.75*jmax).scaled(1.002)
        if key in self.cache:
            old=self.cache[key];self.cache_hits+=1
            # Preserve exact requested distance; the reserve covers roundoff.
            p=Profile(d,old.peak,old.ramp,max(0.,d/old.peak-old.ramp))
            if abs(p.peak*(p.ramp+p.cruise)-d)>1e-7:raise PlanningError('Profile cache distance mismatch')
            return p,baseline.duration
        rec={'distance':d,'vmax':vmax,'amax':amax,'jmax':jmax,'baseline_seconds':baseline.duration,
             'method':self.mode,'accepted':False}
        p=baseline
        if self.mode=='slsqp':
            def obj(x):return d/x[0]+x[1]
            def jac(x):return np.array([-d/x[0]**2,1.])
            def con(x):
                v,r=x;return np.array([d-v*r,amax*r-1.875*v,jmax*r*r-KJ*v])
            def cjac(x):
                v,r=x;return np.array([[-r,-v],[-1.875,amax],[-KJ,2*jmax*r]])
            result=minimize(obj,[baseline.peak,baseline.ramp],jac=jac,method='SLSQP',
                bounds=[(min(1e-5,vmax*.0001),vmax),(1e-6,max(2*baseline.ramp,baseline.duration))],
                constraints=[{'type':'ineq','fun':con,'jac':cjac}],options={'maxiter':100,'ftol':1e-10})
            v,r=map(float,result.x)
            valid=(result.success and min(con(result.x))>=-1e-7 and v>0 and r>0)
            if valid:
                candidate=Profile(d,v,r,max(0.,d/v-r)).scaled(1.002)
                valid=(candidate.peak<=vmax+1e-9 and candidate.max_acc<=amax+1e-9 and
                       candidate.max_jerk<=jmax+1e-9 and abs(candidate.peak*(candidate.ramp+candidate.cruise)-d)<1e-6)
                if valid and candidate.duration<=baseline.duration+1e-8:
                    p=candidate;rec['accepted']=True
            rec.update(solver_success=bool(result.success),solver_message=str(result.message),iterations=int(result.nit))
            if not rec['accepted']:rec['fallback']='verified_conservative_profile'
        rec['actual_seconds']=p.duration;self.records.append(rec);self.cache[key]=p
        return p,baseline.duration


def merge_collinear(legs):
    """Merge only same-direction collinear motion with identical tool semantics.
    Required centres remain on the segment and are validated independently.
    """
    out=[]
    for leg in legs:
        if math.dist(leg['p0'],leg['p1'])<1e-9:continue
        leg=dict(leg)
        if out:
            old=out[-1];a=np.subtract(old['p1'],old['p0']);b=np.subtract(leg['p1'],leg['p0'])
            den=float(np.linalg.norm(a)*np.linalg.norm(b))
            collinear=abs(float(a[0]*b[1]-a[1]*b[0]))<1e-9*max(1.,den) and float(np.dot(a,b))>0
            if old['kind']==leg['kind'] and old['cluster']==leg['cluster'] and math.dist(old['p1'],leg['p0'])<1e-8 and collinear:
                old['p1']=leg['p1'];old['tasks']+=leg.get('tasks',[]);continue
        out.append(leg)
    return out


def build_events(start,legs,robot,cfg,check=lambda:None):
    optimizer=TimeOptimizer(cfg.timing);events=[];t=0.;p=tuple(start);yaw=0.;tool=False;baseline_time=0.
    def add(kind,duration,**kw):
        nonlocal t,p,yaw,baseline_time
        if duration<=1e-12:return
        event={'id':len(events),'kind':kind,'t0':t,'t1':t+duration,'duration':duration,
               'p0':list(p),'p1':list(p),'yaw0':yaw,'yaw1':yaw,'tool_on':False,**kw}
        events.append(event);t+=duration;p=tuple(event['p1']);yaw=float(event['yaw1'])
        baseline_time+=event.get('baseline_seconds',duration)
        if len(events)>cfg.max_events:raise PlanningError('Event budget exceeded')
    def toggle(on):
        nonlocal tool
        if tool==on:return
        tool=on
        wait=robot.tool_spinup_s if on else robot.tool_spindown_s
        add('WAIT',wait,reason='assumed_tool_spinup' if on else 'assumed_tool_spindown',tool_on=False)
    # Initial point needs real service credit, even for a one-cell task.
    toggle(True);add('SERVICE',1.,tool_on=True,reason='initial_stationary_service_assumed_1s',cluster=-1)
    for leg in merge_collinear(legs):
        check()
        if math.dist(p,leg['p0'])>1e-7:raise PlanningError('Discontinuous geometric legs')
        dest=tuple(leg['p1']);heading=math.atan2(dest[1]-p[1],dest[0]-p[0]);delta=wrap(heading-yaw)
        if abs(delta)>1e-7:
            toggle(False);b=robot.track_spacing_m
            limits=(min(robot.yaw_rate_radps,2*robot.track_speed_mps/b),
                    min(robot.yaw_acceleration_radps2,2*robot.track_acceleration_mps2/b),
                    min(robot.yaw_jerk_radps3,2*robot.track_jerk_mps3/b))
            pr,base=optimizer.solve(abs(delta),*limits)
            add('TURN',pr.duration,yaw1=yaw+delta,profile=asdict(pr),baseline_seconds=base,
                reason='align_to_next_certified_straight_segment',cluster=leg['cluster'])
        on=leg['kind']=='WORK';toggle(on)
        vmax=min(robot.work_speed_mps if on else robot.transit_speed_mps,robot.track_speed_mps)
        pr,base=optimizer.solve(math.dist(p,dest),vmax,min(robot.linear_acceleration_mps2,robot.track_acceleration_mps2),
                                min(robot.linear_jerk_mps3,robot.track_jerk_mps3))
        add(leg['kind'],pr.duration,p1=list(dest),tool_on=on,profile=asdict(pr),baseline_seconds=base,
            reason=leg['reason'],cluster=leg['cluster'],tasks=leg.get('tasks',[]))
    toggle(False)
    stats={'mode':cfg.timing,'profile_solves':len(optimizer.records),'cache_hits':optimizer.cache_hits,
           'accepted_numeric_solves':sum(r['accepted'] for r in optimizer.records),
           'fallback_profile_count':sum('fallback' in r for r in optimizer.records),
           'baseline_duration_s':baseline_time,'actual_duration_s':t,
           'time_reduction_ratio':1-t/baseline_time if baseline_time else 0.,
           'scope':'per-primitive constrained peak/ramp optimization, fixed stop-turn-go path',
           'records':optimizer.records}
    return events,stats

COLUMNS=['t','x','y','yaw','v','omega','a','alpha','u_left','u_right','a_left','a_right','tool_on','event_id']

def evaluate_events(events, times, robot):
    """Evaluate the analytical event law without an events-by-samples scan.

    Samples are grouped by event once. The common sorted-time export path is
    linear after event lookup; arbitrary input order is restored before return.
    This matters for dense grid missions with tens of thousands of events.
    """
    times = np.asarray(times, dtype=float).reshape(-1)
    if not events or not len(times):
        return np.empty((0, len(COLUMNS)))
    if not np.all(np.isfinite(times)):
        raise ValueError("Sample times must be finite")
    ts = np.clip(times, 0, events[-1]['t1'])
    ends = np.array([e['t1'] for e in events])
    indices = np.minimum(np.searchsorted(ends, ts, side='right'), len(events)-1)
    permutation = None
    if np.any(indices[1:] < indices[:-1]):
        permutation = np.argsort(indices, kind='stable')
        indices, ts = indices[permutation], ts[permutation]
    boundaries = np.r_[0, np.flatnonzero(indices[1:] != indices[:-1])+1, len(ts)]
    out = np.zeros((len(ts), len(COLUMNS)))
    out[:, 0] = ts
    for left, right in zip(boundaries[:-1], boundaries[1:]):
        idx = int(indices[left]); e = events[idx]
        sl = slice(left, right); t = ts[sl]-e['t0']
        out[sl, 1:3] = e['p0']; out[sl, 3] = e['yaw0']
        v = np.zeros(right-left); w = v.copy(); a = v.copy(); alpha = v.copy()
        if 'curve' in e:
            from .curve_motion import evaluate_curve_event
            value = evaluate_curve_event(e, t, robot)
            out[sl, 1:3] = value['pos']; out[sl, 3] = value['yaw']
            v, w, a, alpha = value['v'], value['omega'], value['a'], value['alpha']
        elif 'profile' in e:
            pr = Profile(**e['profile']); distance, speed, acceleration, _ = pr.at(t)
            if e['kind'] == 'TURN':
                sign = 1 if e['yaw1'] >= e['yaw0'] else -1
                out[sl, 3] = e['yaw0']+sign*distance
                w = sign*speed; alpha = sign*acceleration
            else:
                direction = np.array([math.cos(e['yaw0']), math.sin(e['yaw0'])])
                out[sl, 1:3] = np.asarray(e['p0'])+distance[:, None]*direction
                v = speed; a = acceleration
        out[sl, 4] = v; out[sl, 5] = w
        out[sl, 6] = a; out[sl, 7] = alpha
        out[sl, 8] = v-robot.track_spacing_m*w/2
        out[sl, 9] = v+robot.track_spacing_m*w/2
        out[sl, 10] = a-robot.track_spacing_m*alpha/2
        out[sl, 11] = a+robot.track_spacing_m*alpha/2
        out[sl, 12] = int(e['tool_on']); out[sl, 13] = idx
    if permutation is not None:
        restored = np.empty_like(out); restored[permutation] = out
        return restored
    return out


def sample_events(events,robot,cfg):
    if not events:return np.empty((0,len(COLUMNS))),cfg.sample_dt_s
    duration=events[-1]['t1'];dt=max(cfg.sample_dt_s,duration/max(1,cfg.max_samples-1))
    times=np.arange(0,duration,dt);times=np.append(times,duration)
    return evaluate_events(events,times,robot),dt
