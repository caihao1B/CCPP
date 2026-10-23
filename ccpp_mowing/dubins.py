"""Six-family forward bounded-curvature pose connectors, implemented locally.

Theory: Dubins (1957), as presented in LaValle, Planning Algorithms, 15.3.1.
All six words are evaluated. Obstacles are NOT part of Dubins' optimality
statement: callers must certify each candidate and can reject all candidates.
"""
from __future__ import annotations
import math
from .curves import line,arc,endpoints,wrap


def mod(a):return a%(2*math.pi)


def _words(a,b,d):
    sa,sb,ca,cb,cab=math.sin(a),math.sin(b),math.cos(a),math.cos(b),math.cos(a-b)
    rows=[]
    def add(word,t,p,q):
        if all(math.isfinite(x) and x>=-1e-9 for x in (t,p,q)):rows.append((word,(max(0,t),max(0,p),max(0,q))))
    v=2+d*d-2*cab+2*d*(sa-sb)
    if v>=-1e-10:
        z=math.atan2(cb-ca,d+sa-sb);add('LSL',mod(-a+z),math.sqrt(max(0,v)),mod(b-z))
    v=2+d*d-2*cab+2*d*(sb-sa)
    if v>=-1e-10:
        z=math.atan2(ca-cb,d-sa+sb);add('RSR',mod(a-z),math.sqrt(max(0,v)),mod(-b+z))
    v=-2+d*d+2*cab+2*d*(sa+sb)
    if v>=-1e-10:
        p=math.sqrt(max(0,v));z=math.atan2(-ca-cb,d+sa+sb)-math.atan2(-2.,p)
        add('LSR',mod(-a+z),p,mod(-b+z))
    v=d*d-2+2*cab-2*d*(sa+sb)
    if v>=-1e-10:
        p=math.sqrt(max(0,v));z=math.atan2(ca+cb,d-sa-sb)-math.atan2(2.,p)
        add('RSL',mod(a-z),p,mod(b-z))
    v=(6-d*d+2*cab+2*d*(sa-sb))/8
    if abs(v)<=1+1e-10:
        p=mod(2*math.pi-math.acos(max(-1.,min(1.,v))))
        t=mod(a-math.atan2(ca-cb,d-sa+sb)+p/2);add('RLR',t,p,mod(a-b-t+p))
    v=(6-d*d+2*cab+2*d*(-sa+sb))/8
    if abs(v)<=1+1e-10:
        p=mod(2*math.pi-math.acos(max(-1.,min(1.,v))))
        t=mod(-a-math.atan2(ca-cb,d+sa-sb)+p/2);add('LRL',t,p,mod(b-a-t+p))
    return rows


def candidates(q0,q1,radius):
    if radius<=0:raise ValueError('Dubins requires a positive minimum turning radius')
    if not all(math.isfinite(float(x)) for x in (*q0,*q1,radius)):raise ValueError('Nonfinite Dubins input')
    dx,dy=q1[0]-q0[0],q1[1]-q0[1];theta=math.atan2(dy,dx);d=math.hypot(dx,dy)/radius
    a,b=mod(q0[2]-theta),mod(q1[2]-theta)
    out=[]
    if d<1e-10 and abs(wrap(q1[2]-q0[2]))<1e-10:return []
    for word,sizes in _words(a,b,d):
        p=(float(q0[0]),float(q0[1]));yaw=float(q0[2]);gs=[]
        for mode,z in zip(word,sizes):
            if z<1e-9:continue
            if mode=='S':
                dest=(p[0]+radius*z*math.cos(yaw),p[1]+radius*z*math.sin(yaw));g=line(p,dest)
            else:
                sign=1 if mode=='L' else -1
                c=(p[0]-sign*radius*math.sin(yaw),p[1]+sign*radius*math.cos(yaw))
                g=arc(c,radius,yaw-sign*math.pi/2,sign*z)
            _,dest,_,end_yaw=endpoints(g);p=tuple(dest);yaw=end_yaw;gs.append(g)
        if math.dist(p,q1[:2])>2e-7*max(1.,radius,d) or abs(wrap(yaw-q1[2]))>1e-7:
            # Never return a mislabeled candidate on degenerate numeric input.
            continue
        out.append({'word':word,'length':radius*sum(sizes),'radius':radius,'curves':gs})
    return sorted(out,key=lambda x:x['length'])
