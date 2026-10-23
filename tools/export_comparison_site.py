"""Export actual v2.1 ordered routes, curves and whole-turn video (not generated imagery).
Usage: python tools/export_comparison_site.py --runs comparison_examples --site ../Zehao_Portfolio
Only prevalidated results are read. FFmpeg is optional: --no-video skips video export.
"""
from pathlib import Path
import argparse,json,csv,math,sys,shutil,subprocess,hashlib
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as Patch
from PIL import Image,ImageDraw,ImageFont,ImageColor
from ccpp_mowing.pipeline import validate_saved,write_json
from ccpp_mowing.models import Robot
from ccpp_mowing.geometry import rectangle
from ccpp_mowing.curves import flatten,at,wrap
from ccpp_mowing.motion import evaluate_events

LABELS={'greedy':'Greedy','mst':'MST-Euler','ga':'GA + 2-opt'}

def read(folder,name):return json.loads((folder/name).read_text(encoding='utf-8'))

def load(folder):
    check=validate_saved(folder)
    if not check['valid']:raise ValueError(check['errors'])
    return {'id':folder.name,'events':read(folder,'events.json'),'config':read(folder,'config.json'),
      'robot':read(folder,'robot.json')['simulation_parameters'],'map':read(folder,'map.geojson'),
      'visits':read(folder,'node_visit_order.json'),'raw_legs':read(folder,'raw_legs.json'),
      'metrics':read(folder,'metrics.json'),'optimization':read(folder,'optimization.json'),
      'grid':read(folder,'grid.geojson'),'masks':read(folder,'task_masks.geojson'),
      'validation':check,'audit':read(folder,'continuity_audit.json'),'case':read(folder,'comparison_case.json')}

def groups(d):
    return [{'event':e['id'],'kind':e['kind'],'type':e['curve']['type'],'turn_id':e.get('turn_id'),
             'points':flatten(e['curve'],d['config']['curve_tolerance_m'],.06)[1].tolist()}
            for e in d['events'] if 'curve' in e]

def windows(d):
    groups={}
    for e in d['events']:
        key=e.get('turn_id')
        if key:groups.setdefault(key,[]).append(e)
    out=[]
    for key,ev in groups.items():
        xy=np.vstack([flatten(e['curve'])[1] for e in ev]);a,b=ev[0],ev[-1]
        out.append({'id':key,'t0':a['t0'],'t1':b['t1'],'p0':a['p0'],'p1':b['p1'],
            'yaw0':a['yaw0'],'yaw1':b['yaw1'],'delta':wrap(b['yaw1']-a['yaw0']),
            'bbox':[xy[:,0].min(),xy[:,1].min(),xy[:,0].max(),xy[:,1].max()],
            'word':''.join('S' if e['curve']['type']=='LINE' else ('L' if e['curve']['sweep']>0 else 'R') for e in ev)})
    return out

def polys(feature):
    g=feature['geometry'];return g['coordinates'] if g['type']=='MultiPolygon' else [g['coordinates']]

def scene(ax,d,stage='curve',labels=True):
    for f in d['map']['features']:
        for poly in polys(f):
            ax.add_patch(Patch(poly[0],closed=True,fill=False,hatch='///' if f['properties']['kind']=='obstacle' else None,linewidth=1.2))
    if stage=='grid':
        for f in d['grid']['features']:
            if f['properties']['traversable']:ax.add_patch(Patch(f['geometry']['coordinates'][0],fill=False,linewidth=.45,alpha=.55))
    else:
        xy=np.array([d['raw_legs'][0]['p0']]+[l['p1'] for l in d['raw_legs']])
        ax.plot(*xy.T,'--',linewidth=.8,alpha=.55,label='Ordered route (before curves)')
        if stage=='curve':
            bykind={'WORK':[],'TRANSIT':[]}
            for g in d['curve_groups']:bykind[g['kind']].extend(g['points']+[[np.nan,np.nan]])
            for k,p in bykind.items():
                if p:ax.plot(*np.array(p).T,linewidth=1.2,linestyle='-' if k=='WORK' else ':',label='Work curve' if k=='WORK' else 'Transit / forward loops')
    unique={r['node_id']:(r['x_m'],r['y_m']) for r in d['visits']}
    ax.scatter(*np.array(list(unique.values())).T,s=5,alpha=.55)
    if labels:
        seen=set()
        for r in d['visits'][:18]:
            if r['node_id'] in seen:continue
            seen.add(r['node_id']);ax.annotate(str(r['sequence']),(r['x_m'],r['y_m']),xytext=(2,4),textcoords='offset points',fontsize=7)
    ax.set_aspect('equal');ax.autoscale_view();ax.margins(.04);ax.set_xlabel('x [m]');ax.set_ylabel('y [m]')
    ax.grid(True,alpha=.12)


def picture(d,out,title,stage='curve',window=None,poses=False):
    fig,ax=plt.subplots(figsize=(9.4,6.6),dpi=150);fig.subplots_adjust(left=.08,right=.98,top=.86,bottom=.16)
    scene(ax,d,stage,labels=window is None)
    if window:
        x0,y0,x1,y1=window['bbox'];pad=max(.6,d['robot']['length_m']*.9)
        ax.set_xlim(x0-pad,x1+pad);ax.set_ylim(y0-pad,y1+pad)
        if poses:
            vals=evaluate_events(d['events'],np.linspace(window['t0'],window['t1'],5)[1:-1],Robot(**d['robot']))
            for p in vals:
                poly=rectangle(p[1:3],p[3],d['robot']['length_m'],d['robot']['width_m'])
                ax.add_patch(Patch(poly.exterior.coords,fill=False,linewidth=1.8))
    ax.set_title(title,fontsize=13,pad=14)
    if stage!='grid':ax.legend(loc='upper center',bbox_to_anchor=(.5,-.12),ncol=3,fontsize=8)
    fig.savefig(out,dpi=150);plt.close(fig)


def video(d,out,window,seconds=24,fps=24):
    rb=Robot(**d['robot']);t0,t1=window['t0'],window['t1']
    times=np.linspace(t0,t1,seconds*fps);poses=evaluate_events(d['events'],times,rb)
    fig,ax=plt.subplots(figsize=(10,7),dpi=96);fig.subplots_adjust(left=.08,right=.97,top=.82,bottom=.24)
    scene(ax,d,labels=False);x0,y0,x1,y1=window['bbox'];pad=max(.65,rb.length_m)
    # Keep the entire multi-arc reversal on screen, including intermediate STEER events.
    ax.set_xlim(x0-pad,x1+pad);ax.set_ylim(y0-pad,y1+pad)
    ax.set_title(f"Full forward heading change | {window['word']} | R >= {d['config']['min_turn_radius_m']:.2f} m\nSame planned visit order; no reverse, no chassis pivot",fontsize=12,pad=15)
    fig.canvas.draw();bg=Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[:,:,:3]);W,H=bg.size
    colors=plt.rcParams['axes.prop_cycle'].by_key()['color'];body=ImageColor.getrgb(colors[3]);tail=ImageColor.getrgb(colors[2]);ink=ImageColor.getrgb(plt.rcParams['text.color'])
    font=ImageFont.load_default(size=16)
    for font_path in ('C:/Windows/Fonts/arial.ttf','/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf','/System/Library/Fonts/Supplemental/Arial.ttf'):
        if Path(font_path).exists():
            font=ImageFont.truetype(font_path,16);break
    def screen(points):
        a=ax.transData.transform(np.asarray(points));a[:,1]=H-a[:,1];return [tuple(p) for p in a]
    cmd=['ffmpeg','-hide_banner','-loglevel','error','-y','-f','rawvideo','-pix_fmt','rgb24','-s',f'{W}x{H}','-r',str(fps),'-i','-','-an','-c:v','libx264','-threads','2','-preset','fast','-crf','21','-pix_fmt','yuv420p','-movflags','+faststart',str(out)]
    proc=subprocess.Popen(cmd,stdin=subprocess.PIPE)
    try:
        for i,a in enumerate(poses):
            frame=bg.copy();draw=ImageDraw.Draw(frame)
            poly=rectangle(a[1:3],a[3],rb.length_m,rb.width_m)
            draw.line(screen(poly.exterior.coords),fill=body,width=3)
            tip=a[1:3]+rb.length_m*.66*np.array([math.cos(a[3]),math.sin(a[3])]);draw.line(screen([a[1:3],tip]),fill=body,width=4)
            if i>0:draw.line(screen(poses[:i+1,1:3]),fill=tail,width=2)
            e=d['events'][int(a[13])]
            draw.text((65,H-114),f"t={a[0]:.2f} s   {e['kind']}    v={a[4]:.3f} m/s    yaw rate={a[5]:.3f} rad/s",font=font,fill=ink)
            draw.text((65,H-87),f"Whole turn: {(t1-t0):.1f} simulated seconds; playback {(t1-t0)/seconds:.2f}x",font=font,fill=ink)
            draw.text((65,H-60),'Analytical reference replay. Stops at joins; not closed-loop vehicle simulation.',font=font,fill=ink)
            proc.stdin.write(frame.tobytes())
            if i==len(poses)//2:frame.save(out.with_suffix('.jpg'))
        proc.stdin.close();code=proc.wait(timeout=30)
        if code:raise RuntimeError(f'FFmpeg returned {code}')
    finally:
        if proc.poll() is None:proc.kill();proc.wait()
        plt.close(fig)
    np.savetxt(out.with_suffix('.csv'),poses,delimiter=',',header='t,x,y,yaw,v,omega,a,alpha,left_v,right_v,left_a,right_a,tool,event',comments='')
    return dict(frames=len(poses),fps=fps,video_seconds=seconds,simulation_t0=t0,simulation_t1=t1,turn_id=window['id'],word=window['word'])


def export(runs,site,no_video=False):
    runs=Path(runs);site=Path(site);dat=site/'assets/data';img=site/'assets/images';vid=site/'assets/videos'
    for p in (dat,img,vid):p.mkdir(parents=True,exist_ok=True)
    rows=read(runs,'summary.json');ds={};browser=[];manifest={}
    for row in rows:
        if not row['valid']:raise ValueError(f"Case failed: {row['id']}")
        d=load(runs/row['id']);d['curve_groups']=groups(d);d['turn_windows']=windows(d);ds[d['id']]=d
        fields=('id','kind','t0','t1','duration','p0','p1','yaw0','yaw1','tool_on','steer0','steer1','profile','steer_profile','curve','reason','turn_id')
        b={k:d[k] for k in ('id','config','robot','metrics','map','raw_legs','visits','curve_groups','turn_windows','audit')}
        b['events']=[{k:e[k] for k in fields if k in e} for e in d['events']]
        b['cells']=[{'id':f['properties']['id'],'points':f['geometry']['coordinates'][0]} for f in d['grid']['features'] if f['properties']['traversable']]
        b['declared']=next(f for f in d['masks']['features'] if f['properties']['kind']=='declared')
        b['optimization']={k:d['optimization'][k] for k in ('raw_length_m','heading_loops','cross_row_loops','pivot_fallbacks','curve_counts','time_reduction_ratio')}
        b['optimization']['shape_fallbacks']=sum(bool(r.get('shape_fallback')) for r in d['optimization']['records'])
        b['case']=row;browser.append(b)
        shutil.copy2(runs/d['id']/'node_visit_order.csv',dat/(d['id']+'_visits.csv'))
        manifest[d['id']]={'events_sha256':hashlib.sha256((runs/d['id']/'events.json').read_bytes()).hexdigest(),'order_sha256':row['order_sha256'],'valid':True}
    (dat/'cases.js').write_text('window.CCPP_CASES = '+json.dumps(browser,ensure_ascii=False,separators=(',',':'),allow_nan=False)+';\n',encoding='utf-8')
    columns=['id','grid','solver','targets','planned_visits','raw_length_m','curve_length_m','duration_s','declared_area_percent','original_area_percent','pivots','heading_loops']
    with (dat/'comparison.csv').open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.writer(f);w.writerow(columns)
        for b in browser:
            m=b['metrics'];o=b['optimization'];w.writerow([b['id'],b['config']['grid_type'],b['config']['solver'],m['candidate_centres'],len(b['visits']),o['raw_length_m'],m['path_length_m'],m['trajectory_duration_s'],100*m['declared_area_coverage_ratio'],100*m['original_area_coverage_ratio'],m['pivot_fallbacks'],o['heading_loops']])
    # Independent figures, not subplots. These are actual input grids and actual curves.
    for key,title in [('square_ga','Square grid | GA + 2-opt | constrained curves'),('triangle_ga','Triangle grid | GA + 2-opt | constrained curves')]:
        picture(ds[key],img/(key+'.png'),title)
        picture(ds[key],img/(key+'_grid.png'),title.split('|')[0]+'| same cell area 0.36 m2',stage='grid')
    picture(ds['square_ga'],img/'ordered_route.png','Planned node order -> feasible connected polyline',stage='route')
    d=ds['square_mst'];reversal=next((w for w in d['turn_windows'] if abs(w['delta'])>3.),d['turn_windows'][0])
    picture(d,img/'heading_loop.png','180-degree heading change is a real forward loop',window=reversal,poses=True)
    b=ds['triangle_ga'];e=next(e for e in b['events'] if e.get('curve',{}).get('type')=='BEZIER5')
    xy=flatten(e['curve'])[1];bw={'t0':e['t0'],'t1':e['t1'],'bbox':[xy[:,0].min(),xy[:,1].min(),xy[:,0].max(),xy[:,1].max()]}
    picture(b,img/'bezier_closeup.png','Same node order -> quintic Bezier curve -> timed motion',window=bw,poses=True)
    # Overlay only the SAME first row transition for radius comparison.
    fig,ax=plt.subplots(figsize=(9.4,6.6),dpi=150);fig.subplots_adjust(left=.09,right=.98,top=.85,bottom=.16)
    allxy=[]
    for key in ('car_r060','car_r090'):
        d=ds[key];win=next(w for w in d['turn_windows'] if w['id'].startswith('bridge'))
        xy=np.vstack([flatten(e['curve'])[1] for e in d['events'] if e.get('turn_id')==win['id']]);allxy.extend(xy)
        ax.plot(*xy.T,linewidth=2,label=f"R_min = {d['config']['min_turn_radius_m']:.2f} m")
    ax.set_aspect('equal');ax.grid(True,alpha=.15);ax.legend();ax.set(xlabel='x [m]',ylabel='y [m]',title='Same task order and endpoints | different minimum turning radii\nForward only; no chassis pivot or reverse')
    fig.savefig(img/'radius_comparison.png',dpi=150);plt.close(fig)
    videos={}
    if not no_video:
        videos['heading_loop']=video(ds['square_mst'],vid/'heading_loop.mp4',reversal,seconds=26)
        d=ds['car_r060'];win=next(w for w in d['turn_windows'] if w['id'].startswith('bridge'))
        videos['forward_uturn']=video(d,vid/'forward_uturn.mp4',win,seconds=26)
        write_json(dat/'media_manifest.json',videos)
    write_json(dat/'evidence_manifest.json',manifest)
    print('Exported',len(browser),'integrated cases to',site)
    return browser

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--runs',default='comparison_examples');p.add_argument('--site',required=True);p.add_argument('--no-video',action='store_true');a=p.parse_args();export(a.runs,a.site,a.no_video)
