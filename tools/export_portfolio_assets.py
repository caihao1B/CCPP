"""Build web-ready figures, reference-replay videos and browser data from VERIFIED runs.
No simulated image generation. Matplotlib uses its default color cycle.
Usage: python tools/export_portfolio_assets.py --runs outputs/portfolio --site ../website
"""
from pathlib import Path
import sys,json,math,csv,argparse,subprocess,hashlib,shutil
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from PIL import Image,ImageDraw,ImageFont,ImageColor
from ccpp_mowing.models import Config,Robot
from ccpp_mowing.geometry import World,FieldMap,rectangle
from ccpp_mowing.navigation import Navigator
from ccpp_mowing.curves import flatten,at,curvature
from ccpp_mowing.motion import evaluate_events
from ccpp_mowing.pipeline import validate_saved,write_json
import ccpp_mowing.search as search


def load(folder):
    folder=Path(folder);rd=lambda n:json.loads((folder/n).read_text())
    check=validate_saved(folder)
    if not check['valid']:raise ValueError(check['errors'])
    return dict(folder=folder,metrics=rd('metrics.json'),cfg=rd('config.json'),robot=rd('robot.json')['simulation_parameters'],
                events=rd('events.json'),legs=rd('raw_legs.json'),order=rd('node_visit_order.json'),map=rd('map.geojson'),
                validation=check,optimization=rd('optimization.json'),queries=rd('queries.json'))


def groups(data):
    out=[]
    for e in data['events']:
        if 'curve' not in e:continue
        us,xy=flatten(e['curve'],data['cfg']['curve_tolerance_m'],data['cfg']['curve_step_m'])
        out.append({'kind':e['kind'],'event':e['id'],'type':e['curve']['type'],'points':xy.tolist()})
    return out


def scene(ax,data,raw=True,curve=True):
    for f in data['map']['features']:
        geo=f['geometry'];parts=geo['coordinates'] if geo['type']=='MultiPolygon' else [geo['coordinates']]
        for polygon in parts:
            pts=np.array(polygon[0]);obstacle=f['properties']['kind']=='obstacle'
            ax.add_patch(Polygon(pts,closed=True,fill=False,hatch='////' if obstacle else None,
                                 linewidth=1.4,label=None))
            for hole in polygon[1:]:ax.add_patch(Polygon(hole,closed=True,fill=False,hatch='///'))
    if raw:
        xy=np.array([data['legs'][0]['p0']]+[l['p1'] for l in data['legs']])
        ax.plot(*xy.T,'--',linewidth=.8,alpha=.48,label='Raw connected route')
    if curve:
        bykind={'WORK':[],'TRANSIT':[]}
        for g in groups(data):bykind[g['kind']].extend(g['points']+[[np.nan,np.nan]])
        for kind,xy in bykind.items():
            if xy:ax.plot(*np.asarray(xy).T,linewidth=1.4 if kind=='WORK' else .9,
                          linestyle='-' if kind=='WORK' else ':',label='Optimized work curve' if kind=='WORK' else 'Transit / revisit')
    ax.set_aspect('equal',adjustable='box');ax.autoscale_view();ax.margins(.04)
    ax.set_xlabel('x [m]');ax.set_ylabel('y [m]');ax.grid(True,alpha=.15)


def figure(data,title,figsize=(11,7.2)):
    fig,ax=plt.subplots(figsize=figsize,dpi=150)
    fig.subplots_adjust(left=.08,right=.97,top=.85,bottom=.16)
    scene(ax,data);ax.set_title(title,fontsize=14,pad=18)
    ax.legend(loc='lower center',bbox_to_anchor=(.5,-.20),ncol=3,fontsize=8)
    return fig,ax


def savefig(fig,path):
    fig.savefig(path,dpi=170)
    fig.savefig(path.with_suffix('.svg'))
    plt.close(fig)


def build_astar(data,out):
    cfg=Config.from_dict(data['cfg']);rb=Robot(**data['robot']);w=World(FieldMap.from_geojson(data['map']),rb,cfg.manual_setback_m)
    blocked=[q for q in data['queries'] if q['actual']=='astar' and not w.segment_free(q['start'],q['goal'])]
    # Prefer an actual mid-field obstacle crossing, not a barely clipped margin.
    q=max(blocked,key=lambda q:q.get('primary_stats',{}).get('expanded',0) if abs(q['start'][1]-q['goal'][1])<.01 else -1)
    nav=Navigator(w,cfg);captured={};original=search.astar_graph
    def traced(neighbors,start,goal,positions,*args,**kwargs):
        ids,cost,expanded=original(neighbors,start,goal,positions,*args,**kwargs)
        captured.update(ids=ids,cost=cost,expanded=expanded,positions=positions)
        return ids,cost,expanded
    search.astar_graph=traced
    try:used=nav.route(q['start'],q['goal'])
    finally:search.astar_graph=original
    reference=nav.road.query(q['start'],q['goal'],heuristic=False)
    assert reference.success and abs(captured['cost']-reference.stats['cost'])<1e-8
    pos=captured['positions'];ids=captured['ids'];expanded=captured['expanded']
    payload={'source_case':'obstacle_astar_bezier','purpose':'rerun a logged real blocked coverage connector',
             'start':q['start'],'goal':q['goal'],'direct_line_safe':False,
             'astar_cost_m':captured['cost'],'dijkstra_cost_m':reference.stats['cost'],
             'expanded_states':len(expanded),'dijkstra_expanded_states':reference.stats['expanded'],
             'path_node_ids':ids,'path':[pos[k] for k in ids],
             'expansion_order':[{'node_id':k,'xy':pos[k]} for k in expanded],
             'used_connector_after_shortcut':used,'road_positions':[pos[k] for k in sorted(pos) if k>=0],
             'map':data['map'],'robot':data['robot'],
             'scope':'finite weighted graph shortest route; no global coverage/trajectory optimum claim'}
    write_json(out/'astar_evidence.json',payload)
    with (out/'astar_node_sequence.csv').open('w',encoding='utf-8-sig',newline='') as f:
        wr=csv.writer(f);wr.writerow(['sequence','navigation_node_id','x_m','y_m'])
        wr.writerows((i+1,k,*pos[k]) for i,k in enumerate(ids))
    return payload


def render_video(data,out,closeup=False,seconds=16,fps=24):
    rb=Robot(**data['robot']);events=data['events'];total=events[-1]['t1']
    first=next(e for e in events if e.get('curve',{}).get('type') not in (None,'LINE'))
    t0=max(0.,first['t0']-3) if closeup else 0.
    t1=min(total,first['t1']+3) if closeup else total
    frames=seconds*fps;times=np.linspace(t0,t1,frames);poses=evaluate_events(events,times,rb)
    fig,ax=plt.subplots(figsize=(10,7),dpi=96)
    fig.subplots_adjust(left=.08,right=.97,top=.80,bottom=.23)
    scene(ax,data)
    if closeup:
        _,xy=flatten(first['curve']);mid=(xy.max(axis=0)+xy.min(axis=0))/2
        half=max(1.35,.5*np.ptp(xy,axis=0).max()+1.0)
        ax.set_xlim(mid[0]-1.35*half,mid[0]+1.35*half);ax.set_ylim(mid[1]-half,mid[1]+half)
    else:
        seen=set()
        for row in data['order'][:20]:
            if row['node_id'] in seen:continue
            seen.add(row['node_id']);ax.annotate(str(row['sequence']),(row['x_m'],row['y_m']),xytext=(3,4),textcoords='offset points',fontsize=7)
    model=data['cfg']['motion_model'];method=data['cfg']['trajectory_method']
    ax.set_title(f'{model} | {method}\nAnalytical reference replay - not measured tracking',fontsize=12,pad=18)
    fig.canvas.draw();background=Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[:,:,:3])
    width,height=background.size
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',15)
    default_colors=plt.rcParams['axes.prop_cycle'].by_key()['color'];body_color=ImageColor.getrgb(default_colors[3])
    tail_color=ImageColor.getrgb(default_colors[2]);ink=ImageColor.getrgb(plt.rcParams['text.color'])
    def screen(xy):
        pts=ax.transData.transform(np.asarray(xy));pts[:,1]=height-pts[:,1];return [tuple(p) for p in pts]
    cmd=['ffmpeg','-hide_banner','-loglevel','error','-y','-f','rawvideo','-pix_fmt','rgb24',
         '-s',f'{width}x{height}','-r',str(fps),'-i','-','-an','-c:v','libx264','-threads','2','-preset','fast',
         '-crf','23','-pix_fmt','yuv420p','-movflags','+faststart',str(out)]
    p=subprocess.Popen(cmd,stdin=subprocess.PIPE)
    for i,a in enumerate(poses):
        image=background.copy();draw=ImageDraw.Draw(image)
        poly=rectangle(a[1:3],a[3],rb.length_m,rb.width_m)
        draw.line(screen(np.array(poly.exterior.coords)),fill=body_color,width=3)
        tip=a[1:3]+rb.length_m*.65*np.array([math.cos(a[3]),math.sin(a[3])])
        draw.line(screen([a[1:3],tip]),fill=body_color,width=4)
        idx=max(0,i-24)
        if i>idx:draw.line(screen(poses[idx:i+1,1:3]),fill=tail_color,width=3)
        e=events[int(a[-1])]
        draw.text((65,height-102),f't = {a[0]:.2f} / {total:.2f} s    v = {a[4]:.3f} m/s    yaw rate = {a[5]:.3f} rad/s',font=font,fill=ink)
        draw.text((65,height-77),f'{e["kind"]} | R_min = {data["cfg"]["min_turn_radius_m"]:.2f} m | playback {(t1-t0)/seconds:.1f}x',font=font,fill=ink)
        draw.text((65,height-52),'Labels = planned task sequence; curve completion uses cutter-footprint service.',font=font,fill=ink)
        p.stdin.write(image.tobytes())
        if i==frames//2:image.save(out.with_suffix('.jpg'))
    p.stdin.close();code=p.wait(timeout=30);plt.close(fig)
    if code:raise RuntimeError(f'ffmpeg failed {code}')
    np.savetxt(out.with_suffix('.csv'),poses,delimiter=',',header='t,x,y,yaw,v,omega,a,alpha,u_left,u_right,a_left,a_right,tool_on,event_id',comments='')
    return {'frames':frames,'fps':fps,'video_seconds':seconds,'simulation_range_s':[float(t0),float(t1)]}


def export(runs,site,skip_video=False):
    runs=Path(runs);site=Path(site);assets=site/'assets';img=assets/'images';vid=assets/'videos';dat=assets/'data'
    for d in (img,vid,dat):d.mkdir(parents=True,exist_ok=True)
    cases=json.loads((runs/'portfolio_cases.json').read_text());datasets={}
    for case in cases:
        if not case['valid']:continue
        key=case['id'];d=load(runs/key);datasets[key]=d
        f,ax=figure(d,case['title']+'\nOriginal task order is preserved as an explicit obligation')
        seen=set()
        for row in d['order'][:24]:
            if row['node_id'] in seen:continue
            seen.add(row['node_id']);ax.scatter([row['x_m']],[row['y_m']],s=15)
            ax.annotate(str(row['sequence']),(row['x_m'],row['y_m']),xytext=(3,4),textcoords='offset points',fontsize=8)
        f.text(.08,.025,'Labels: first 24 planned visits (repeats in CSV). Area coverage and target completion are separate.',fontsize=9)
        savefig(f,img/f'{key}.png')
        for name in ('node_visit_order.csv','node_visit_order.json','config.json','robot.json','metrics.json','validation.json'):
            shutil.copy2(runs/key/name,dat/f'{key}_{name}')
    compact=datasets['compact_greedy_astar'];first=next(e for e in compact['events'] if e.get('curve',{}).get('type')=='BEZIER5')
    f,ax=figure(compact,'Quintic Bezier corner: raw route, continuous curve and body poses')
    us,xy=flatten(first['curve']);mid=(xy.max(axis=0)+xy.min(axis=0))/2
    ax.set_xlim(mid[0]-1.35,mid[0]+1.35);ax.set_ylim(mid[1]-1.,mid[1]+1.)
    poses=evaluate_events(compact['events'],first['t0']+np.array([.18,.5,.82])*first['duration'],Robot(**compact['robot']))
    for a in poses:
        poly=rectangle(a[1:3],a[3],compact['robot']['length_m'],compact['robot']['width_m'])
        ax.add_patch(Polygon(np.array(poly.exterior.coords),closed=True,fill=False,linewidth=1.8))
    f.text(.08,.025,'Positions and headings come from the SAME analytical law used for animation and trajectory CSV.',fontsize=9)
    savefig(f,img/'curve_closeup.png')
    astar=build_astar(datasets['obstacle_astar_bezier'],dat)
    f,ax=plt.subplots(figsize=(10,7.2),dpi=150);f.subplots_adjust(bottom=.17,top=.83)
    scene(ax,datasets['obstacle_astar_bezier'],raw=False,curve=False)
    obstacle_data=datasets['obstacle_astar_bezier']
    wrld=World(FieldMap.from_geojson(obstacle_data['map']),Robot(**obstacle_data['robot']))
    for poly in ([wrld.cspace] if wrld.cspace.geom_type=='Polygon' else wrld.cspace.geoms):
        for ring in poly.interiors:
            boundary=np.asarray(ring.coords);ax.plot(*boundary.T,':',linewidth=1.,alpha=.8,label='Conservative centre clearance')
    road=np.asarray(astar['road_positions']);ax.scatter(*road.T,s=5,alpha=.15,label='Safe navigation nodes')
    expanded=np.asarray([a['xy'] for a in astar['expansion_order']]);ax.scatter(*expanded.T,s=24,alpha=.5,label='A* expanded states')
    p=np.array(astar['path']);ax.plot(*p.T,'o-',linewidth=2,label='A* graph shortest path')
    direct=np.array([astar['start'],astar['goal']]);ax.plot(*direct.T,'--',linewidth=1.3,label='Blocked direct connector')
    for i,xy in enumerate(p,1):ax.annotate(str(i),xy,xytext=(5,5),textcoords='offset points',fontsize=10)
    pad=1.2;ax.set_xlim(p[:,0].min()-pad,p[:,0].max()+pad);ax.set_ylim(p[:,1].min()-pad,p[:,1].max()+pad)
    ax.set_title(f'ACTUAL A* DETOUR | {astar["expanded_states"]} expanded states\nGraph cost {astar["astar_cost_m"]:.3f} m = Dijkstra {astar["dijkstra_cost_m"]:.3f} m',pad=15)
    ax.legend(loc='lower center',bbox_to_anchor=(.5,-.24),ncol=2,fontsize=8)
    savefig(f,img/'astar_detour.png')
    # Same raw task route and vehicle; only R_min is changed.
    f,ax=plt.subplots(figsize=(10,7.2),dpi=150);f.subplots_adjust(bottom=.16,top=.84)
    d0=datasets['forward_dubins_r060'];scene(ax,d0,raw=True,curve=False)
    for key in ('forward_dubins_r060','forward_dubins_r090'):
        d=datasets[key];first=next(e for e in d['events'] if e.get('curve',{}).get('type')=='ARC')
        # All primitives in the same first U-turn, including its internal straight segment.
        start=first['id'];pts=[]
        for e in d['events'][start:]:
            if 'curve' in e and e.get('reason') != first.get('reason'):break
            if 'curve' in e:pts.extend(flatten(e['curve'])[1].tolist())
        q=np.array(pts);ax.plot(*q.T,linewidth=2,label=f'R_min = {d["cfg"]["min_turn_radius_m"]:.2f} m')
    first=next(e for e in d0['events'] if e.get('curve',{}).get('type')=='ARC');center=np.array(first['p0'])
    ax.set_xlim(center[0]-1.6,center[0]+3.);ax.set_ylim(center[1]-1.8,center[1]+3.)
    ax.set_title('Same ordered task route, different minimum turning radii\nForward-only: no chassis pivot, standstill steering is explicitly modeled',pad=16)
    ax.legend(loc='lower center',bbox_to_anchor=(.5,-.21),ncol=3,fontsize=9)
    savefig(f,img/'radius_comparison.png')
    # Certified ratios, not sparse measured maxima.
    ratios=datasets['forward_dubins_r060']['validation']['maximum_continuous_bound_ratios']
    f,ax=plt.subplots(figsize=(10,5.4),dpi=150);f.subplots_adjust(left=.10,right=.97,bottom=.22,top=.82)
    keys=list(ratios);ax.bar(range(len(keys)),[ratios[k] for k in keys]);ax.axhline(1,linestyle='--',label='Constraint limit')
    ax.set_xticks(range(len(keys)),keys,rotation=35,ha='right');ax.set_ylim(0,1.14)
    ax.set_ylabel('Certified bound / configured limit');ax.set_title('Forward-only R=0.60 m | continuous motion-bound audit',pad=16);ax.legend()
    f.text(.1,.025,'Conservative analytical bounds; not manufacturer data. Track values are equivalent-drive envelopes.',fontsize=9)
    savefig(f,img/'motion_limits.png')
    video_rows={}
    if skip_video and (dat/'media_manifest.json').exists():
        video_rows=json.loads((dat/'media_manifest.json').read_text())['videos']
    if not skip_video:
        for key,name,closeup,seconds in [('compact_greedy_astar','compact_full',False,24),
                                       ('compact_greedy_astar','bezier_closeup',True,16),
                                       ('forward_dubins_r060','dubins_closeup',True,18),
                                       ('obstacle_astar_bezier','obstacle_full',False,24)]:
            video_rows[name]=render_video(datasets[key],vid/(name+'.mp4'),closeup,seconds)
            print('video',name,flush=True)
    browser=[]
    for case in cases:
        if not case['valid']:continue
        key=case['id'];d=datasets[key]
        browser.append({'id':key,'title':case['title'],'metrics':d['metrics'],'config':d['cfg'],'robot':d['robot'],
                        'events':d['events'],'map':d['map'],'raw_legs':d['legs'],'visits':d['order'],
                        'curve_groups':groups(d),'validation':d['validation'],
                        'actual_blocked_astar_queries':case['actual_blocked_astar_queries'],
                        'astar_non_cached_queries':sum(q['actual']=='astar' and not q.get('cache_hit') for q in d['queries'])})
    (dat/'demo-data.js').write_text('window.CCPP_DEMOS = '+json.dumps(browser,ensure_ascii=False,separators=(',',':'),allow_nan=False)+';\nwindow.CCPP_ASTAR = '+json.dumps(astar,ensure_ascii=False,separators=(',',':'))+';\n',encoding='utf-8')
    write_json(dat/'media_manifest.json',{'source':'fresh verified v2.0.1 runs','no_hardware_claim':True,'videos':video_rows,
               'run_inputs':{c['id']:hashlib.sha256((runs/c['id']/'events.json').read_bytes()).hexdigest() for c in cases if c['valid']}})
    print('Exported',len(browser),'verified cases to',site)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--runs',required=True);p.add_argument('--site',required=True);p.add_argument('--skip-video',action='store_true');a=p.parse_args()
    export(a.runs,a.site,a.skip_video)
