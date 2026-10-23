"""Render the SAME analytical event law used by the GUI to an offline GIF."""
from __future__ import annotations
from pathlib import Path
import numpy as np
from PIL import Image
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.patches import Polygon,Circle
from .snapshot import load_view
from .motion import evaluate_events
from .curve_motion import curve_diagnostics
from .geometry import rectangle
from .visualization import draw_scene,event_polyline


def first_curve(events):
    return next((e for e in events if 'curve' in e and e['curve']['type']!='LINE'),None) or next((e for e in events if e['kind']=='TURN'),None)


def render_gif(folder,out,whole=False,frames=90):
    if not 20<=frames<=360:raise ValueError('GIF frames must be 20..360')
    result=load_view(folder);duration=result.events[-1]['t1'];turn=first_curve(result.events)
    if not whole and turn:
        t0=max(0.,turn['t0']-3.);t1=min(duration,turn['t1']+3.)
    else:t0,t1=0.,duration
    values=evaluate_events(result.events,np.linspace(t0,t1,frames),result.robot)
    diagnostics=curve_diagnostics(result.events,values[:,0],result.robot,result.config)
    fig=Figure(figsize=(8,6.3),dpi=100);canvas=FigureCanvasAgg(fig);ax=fig.add_subplot(111)
    fig.subplots_adjust(left=.10,right=.97,top=.78,bottom=.18)
    draw_scene(ax,result,show_grid=False,show_clusters=False)
    if not whole and turn:
        xy=np.asarray(event_polyline(turn));pad=max(1.3,result.robot.length_m)
        center=(xy.max(axis=0)+xy.min(axis=0))/2
        half=max(float(np.ptp(xy,axis=0).max())/2+pad,2.)
        ax.set_xlim(center[0]-half,center[0]+half);ax.set_ylim(center[1]-half*.78,center[1]+half*.78)
    method=result.metrics.get('trajectory_method','stop_turn_go');model=result.metrics.get('motion_model','differential')
    ax.set_title(f'{method} | {model}\nAnalytical reference replay; not measured tracking',fontsize=12)
    body=Polygon(np.zeros((4,2)),closed=True,fill=False,linewidth=2.5,zorder=20);ax.add_patch(body)
    cutter=Circle((0,0),result.robot.cutter_radius,fill=False,linestyle=':',linewidth=1.5,zorder=21);ax.add_patch(cutter)
    heading,=ax.plot([],[],linewidth=2.5,zorder=22)
    trail,=ax.plot([],[],linewidth=2.,alpha=.7,zorder=19)
    front,=ax.plot([],[],linewidth=3,zorder=23)
    text=fig.text(.5,.97,'',ha='center',va='top',fontsize=10)
    pictures=[]
    for i,(a,d) in enumerate(zip(values,diagnostics)):
        p=a[1:3];yaw=a[3];body.set_xy(np.array(rectangle(p,yaw,result.robot.length_m,result.robot.width_m).exterior.coords))
        cutter.center=p;cutter.set_visible(bool(a[12]))
        heading.set_data([p[0],p[0]+.75*np.cos(yaw)],[p[1],p[1]+.75*np.sin(yaw)])
        trail.set_data(values[:i+1,1],values[:i+1,2])
        tip=p+.36*np.array([np.cos(yaw),np.sin(yaw)]);steer=yaw+d[2]
        front.set_data([tip[0]-.16*np.cos(steer),tip[0]+.16*np.cos(steer)],[tip[1]-.16*np.sin(steer),tip[1]+.16*np.sin(steer)])
        front.set_visible(model=='bounded_curvature')
        e=result.events[int(a[13])]
        text.set_text(f"t = {a[0]:.2f} s | {e['kind']} | v = {a[4]:.3f} m/s | omega = {a[5]:.3f} rad/s\n"
                      f"kappa = {d[1]:.3f} 1/m | a_lat = {d[4]:.3f} m/s2 | R_min = {result.config.min_turn_radius_m:.2f} m")
        canvas.draw();pictures.append(Image.fromarray(np.asarray(canvas.buffer_rgba())[:,:,:3].copy()).quantize(colors=192))
    out=Path(out);out.parent.mkdir(parents=True,exist_ok=True)
    pictures[0].save(out,save_all=True,append_images=pictures[1:],duration=70,loop=0,optimize=False,disposal=2)
    return str(out.resolve())
