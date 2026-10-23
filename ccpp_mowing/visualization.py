"""Matplotlib diagnostics. No simulated measurements are labelled as hardware data."""
from __future__ import annotations
from pathlib import Path
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.patches import PathPatch
from matplotlib.path import Path as MplPath
from shapely.geometry.polygon import orient
from .geometry import parts, rectangle
from .motion import sample_events


def polygon_patch(p, **kwargs):
    p=orient(p,sign=1.)
    rings=[np.asarray(p.exterior.coords)]+[np.asarray(r.coords) for r in p.interiors]
    return PathPatch(MplPath.make_compound_path(*[MplPath(r) for r in rings]),**kwargs)


def event_polyline(event,step=.06,tolerance=.002):
    if 'curve' not in event:return [event['p0'],event['p1']]
    from .curves import flatten
    return flatten(event['curve'],tolerance,step)[1].tolist()


def draw_scene(ax,result,show_grid=True,show_clusters=True,show_paths=True,show_raw=True):
    world=result.world;ax.clear()
    for i,p in enumerate(parts(world.map.field)):
        ax.plot(*p.exterior.xy,linewidth=1.8,label='Field boundary' if i==0 else None)
    for i,p in enumerate(parts(world.free)):
        for ring in p.interiors:ax.fill(*ring.xy,alpha=.20,hatch='///')
    for obstacle in world.map.obstacles:
        for p in parts(obstacle):ax.fill(*p.exterior.xy,alpha=.22,hatch='///')
    if show_grid and len(result.grid.cells)<6000:
        lc=LineCollection([list(c.polygon.exterior.coords) for c in result.grid.cells],linewidths=.25,alpha=.2)
        ax.add_collection(lc)
    if show_clusters:
        for cid,group in enumerate(result.order.clusters):
            ps=np.asarray([result.grid.positions[u] for u in group]);ax.scatter(ps[:,0],ps[:,1],s=3,alpha=.32)
            if len(result.order.clusters)<=70:
                centre=ps.mean(axis=0);ax.text(*centre,str(cid),fontsize=6,ha='center')
    if show_paths:
        if show_raw and any('curve' in e for e in result.events) and getattr(result,'legs',None):
            ax.add_collection(LineCollection([[e['p0'],e['p1']] for e in result.legs],
                linewidths=.7,linestyles=':',alpha=.40,label='Raw polyline (reference only)'))
        for kind,style,label in [('WORK','-','Work path'),('TRANSIT','--','Transit / revisit')]:
            segs=[event_polyline(e) for e in result.events if e['kind']==kind]
            if segs:ax.add_collection(LineCollection(segs,linewidths=.85,linestyles=style,label=label,alpha=.85))
    for i,p in enumerate(parts(result.areas['original_remaining'])):
        ax.add_patch(polygon_patch(p,alpha=.16,hatch='...',label='Manual residual' if i==0 else None))
    if result.events:
        p=result.events[0]['p0'];ax.scatter(*p,marker='^',s=65,label='Start / home',zorder=8)
    m=result.metrics
    motion_text = (f"{m.get('motion_model')} | {m.get('trajectory_method')} | R >= {m.get('minimum_turn_radius_m',0):.2f} m\n"
                   if m.get('trajectory_method','stop_turn_go')!='stop_turn_go' else '')
    service_text='cutter-covered targets' if m.get('service_rule')=='cutter_footprint' else 'centres passed'
    ax.set(title=motion_text+f"{m['grid'].upper()} | {m['intra_solver']} / {m['actual_inter_solver']} | {m['planner']}\n"
                 f"{m['serviced_centres']}/{m['candidate_centres']} {service_text}, {m['cluster_count']} clusters; "
                 f"declared area {100*(m['declared_area_coverage_ratio'] or 0):.2f}%\n"
                 f"{m['path_length_m']:.1f} m; {m['trajectory_duration_s']/3600:.2f} h simulated; "
                 f"manual {m['manual_handover_area_m2']:.2f} m²",
           xlabel='x [m]',ylabel='y [m]')
    ax.set_aspect('equal');x0,y0,x1,y1=world.bounds;pad=max(1.,.03*max(x1-x0,y1-y0))
    ax.set_xlim(x0-pad,x1+pad);ax.set_ylim(y0-pad,y1+pad)
    ax.legend(loc='upper center',bbox_to_anchor=(.5,-.09),ncol=3,fontsize=8)


def save_plots(result,out):
    out=Path(out)
    fig,ax=plt.subplots(figsize=(12,9));draw_scene(ax,result);fig.tight_layout();fig.savefig(out/'coverage.png',dpi=150);plt.close(fig)
    from .motion import evaluate_events
    duration=result.events[-1]['t1'];sample=evaluate_events(result.events,np.linspace(0,duration,min(15000,max(1000,int(duration/.5)))),result.robot)
    fig,ax=plt.subplots(figsize=(12,4));ax.plot(sample[:,0]/60,sample[:,4],linewidth=.6,label='Planned chassis speed')
    ax.axhline(result.robot.transit_speed_mps,linestyle='--',label='Transit bound')
    ax.set(xlabel='Simulated time [min]',ylabel='Speed [m/s]',title='Finite-rate offline motion — not measured tracking')
    ax.legend();fig.tight_layout();fig.savefig(out/'speed.png',dpi=140);plt.close(fig)
    turn=next((e for e in result.events if 'curve' in e and e['curve']['type']!='LINE'),None) or next((e for e in result.events if e['kind']=='TURN'),None)
    if turn:
        sample=evaluate_events(result.events,np.linspace(max(0,turn['t0']-2),min(duration,turn['t1']+2),600),result.robot)
        fig,ax=plt.subplots(figsize=(10,4));ax.plot(sample[:,0]-turn['t0'],sample[:,8],label='Left track')
        ax.plot(sample[:,0]-turn['t0'],sample[:,9],label='Right track')
        ax.set(xlabel='Time from turn start [s]',ylabel='Track speed [m/s]',title='First curved turn / pivot: planned equivalent wheel speeds')
        ax.legend();fig.tight_layout();fig.savefig(out/'turn_tracks.png',dpi=140);plt.close(fig)

    if result.config.trajectory_method!='stop_turn_go':
        from .curve_motion import curve_diagnostics
        # Curvature peaks are sampled per primitive, not at sparse long-mission
        # clock intervals. Constraint certificates remain sampling-independent.
        chunks=[np.linspace(e['t0'],e['t1'],35,endpoint=False) for e in result.events if 'curve' in e]
        times=np.concatenate(chunks) if chunks else np.array([])
        if len(times):
            diag=curve_diagnostics(result.events,times,result.robot,result.config)
            fig,ax=plt.subplots(figsize=(12,4));ax.plot(diag[:,0]/60,diag[:,1],linewidth=.6,label='Moving-path curvature')
            bound=1/result.config.min_turn_radius_m
            ax.axhline(bound,linestyle='--',label='+/- 1 / R_min');ax.axhline(-bound,linestyle='--')
            ax.set(xlabel='Simulated time [min]',ylabel='Curvature [1/m]',title='Geometric curvature; bounds checked continuously, not from this plot')
            ax.legend();fig.tight_layout();fig.savefig(out/'curvature.png',dpi=140);plt.close(fig)
            fig,ax=plt.subplots(figsize=(12,4));ax.plot(diag[:,0]/60,diag[:,4],linewidth=.6,label='Lateral acceleration')
            ax.axhline(result.config.lateral_acceleration_mps2,linestyle='--',label='Lateral acceleration bound')
            ax.axhline(-result.config.lateral_acceleration_mps2,linestyle='--')
            ax.set(xlabel='Simulated time [min]',ylabel='Lateral acceleration [m/s²]',title='Curved motion: a_lateral = curvature × speed²')
            ax.legend();fig.tight_layout();fig.savefig(out/'lateral_acceleration.png',dpi=140);plt.close(fig)
