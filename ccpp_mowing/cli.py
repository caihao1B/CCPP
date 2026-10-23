"""Public CLI; all paths are explicit, all non-success states have an exit code."""
from __future__ import annotations
import argparse, dataclasses, json, sys, traceback
from pathlib import Path
from . import __version__
from .models import Config, Robot, SOLVERS, INTER_SOLVERS, PLANNERS, Cancelled, MOTION_MODELS, TRAJECTORY_METHODS
from .maps import generate, SHAPES, DENSITIES, DISTRIBUTIONS
from .geometry import FieldMap
from .pipeline import run, save_result, validate_saved, write_json, SCHEMA


def parser():
    p=argparse.ArgumentParser(description='CCPP Mowing Planner: pure-Python offline coverage research')
    p.add_argument('--version',action='version',version=__version__)
    sub=p.add_subparsers(dest='command',required=True)
    g=sub.add_parser('generate',help='Generate a metric synthetic GeoJSON field')
    r=sub.add_parser('plan',help='Plan, time-parameterize, independently validate and export')
    for q in (g,r):
        q.add_argument('--shape',choices=SHAPES,default='rectangle');q.add_argument('--width',type=float,default=18.)
        q.add_argument('--height',type=float,default=14.);q.add_argument('--density',choices=DENSITIES,default='medium')
        q.add_argument('--distribution',choices=DISTRIBUTIONS,default='uniform');q.add_argument('--seed',type=int,default=None)
    g.add_argument('--out',required=True)
    r.add_argument('--map',help='GeoJSON in local metres; omitting it uses the seeded generator')
    r.add_argument('--robot',help='Robot JSON: flat SI parameters or a saved robot.json simulation_parameters object')
    r.add_argument('--config');r.add_argument('--grid',choices=('square','triangle'))
    r.add_argument('--intra',choices=SOLVERS);r.add_argument('--inter',choices=INTER_SOLVERS)
    r.add_argument('--partition',choices=('none','bfs','kmeans'));r.add_argument('--planner',choices=PLANNERS)
    r.add_argument('--cell-area',type=float);r.add_argument('--cluster-size',type=int)
    r.add_argument('--motion-model',choices=MOTION_MODELS)
    r.add_argument('--trajectory',choices=TRAJECTORY_METHODS)
    r.add_argument('--min-radius',type=float)
    r.add_argument('--manual-setback',type=float)
    r.add_argument('--max-metric-nodes',type=int)
    r.add_argument('--curve-step',type=float)
    r.add_argument('--service-rule',choices=('cutter_footprint','centre_exact'))
    r.add_argument('--turn-policy',choices=('legacy_pivot','loop_then_pivot','loops_only'))
    r.add_argument('--no-pivot-fallback',action='store_true')
    r.add_argument('--timing',choices=('slsqp','conservative'));r.add_argument('--strict',action='store_true')
    r.add_argument('--out',default='outputs/latest');r.add_argument('--overwrite',action='store_true')
    r.add_argument('--no-plots',action='store_true');r.add_argument('--no-samples',action='store_true')
    v=sub.add_parser('validate',help='Independently recompute a saved experiment')
    v.add_argument('folder')
    clip=sub.add_parser('render',help='Render an analytical curve-following GIF; no FFmpeg required')
    clip.add_argument('folder');clip.add_argument('--out',required=True)
    clip.add_argument('--whole',action='store_true');clip.add_argument('--frames',type=int,default=90)
    sub.add_parser('gui',help='Open the Tk/Matplotlib desktop application')
    return p


def main(argv=None):
    args=parser().parse_args(argv)
    if args.command=='render':
        try:
            from .animation_export import render_gif
            print(render_gif(args.folder,args.out,whole=args.whole,frames=args.frames));return 0
        except Exception as e:print('GIF export failed: '+str(e),file=sys.stderr);return 2
    if args.command=='gui':
        from .gui import launch
        launch();return 0
    if args.command=='validate':
        try:r=validate_saved(args.folder);print(json.dumps(r,ensure_ascii=False,allow_nan=False));return 0 if r['valid'] else 2
        except Exception as e:print(f'Validation refused: {e}',file=sys.stderr);return 2
    if args.command=='generate':
        try:
            m=generate(args.shape,args.width,args.height,args.density,args.distribution,7 if args.seed is None else args.seed)
            out=Path(args.out);out.parent.mkdir(parents=True,exist_ok=True);write_json(out,m.original_geojson)
            print(str(out.resolve()));return 0
        except Exception as e:print(f'Map generation failed: {e}',file=sys.stderr);return 2
    out=Path(args.out).resolve()
    if out.exists() and any(p.name not in ('request.json','cancel.flag','progress.json') for p in out.iterdir()) and not args.overwrite:
        print(f'Output is not empty: {out}. Choose a new folder or explicitly pass --overwrite.',file=sys.stderr);return 2
    out.mkdir(parents=True,exist_ok=True)
    if args.overwrite:
        # Explicit overwrite only removes known generated regular files; no recursive deletes.
        for p in out.iterdir():
            if p.is_file() and p.suffix.lower() in ('.json','.csv','.png','.txt'):p.unlink()
    write_json(out/'status.json',{'schema':SCHEMA,'status':'running'})
    try:
        cfg=Config.from_dict(json.loads(Path(args.config).read_text(encoding='utf-8'))) if args.config else Config()
        changes={}
        for arg,name in [('grid','grid_type'),('intra','solver'),('inter','cluster_solver'),('partition','partition'),
                         ('planner','planner'),('cell_area','cell_area'),('cluster_size','cluster_size'),('timing','timing'),('seed','seed'),
                         ('motion_model','motion_model'),('trajectory','trajectory_method'),('min_radius','min_turn_radius_m'),
                         ('manual_setback','manual_setback_m'),('max_metric_nodes','max_metric_nodes'),
                         ('curve_step','curve_step_m'),('service_rule','curve_service_rule'),('turn_policy','turn_policy')]:
            v=getattr(args,arg,None)
            if v is not None:changes[name]=v
        if args.no_pivot_fallback:changes['curve_allow_pivot_fallback']=False
        if args.strict:changes.update(fallback_policy='strict',allow_partial=False)
        cfg=dataclasses.replace(cfg,**changes).validate()
        field=FieldMap.load(args.map) if args.map else generate(args.shape,args.width,args.height,args.density,args.distribution,cfg.seed)
        def progress(stage,fraction,text):
            d={'stage':stage,'fraction':fraction,'message':text};write_json(out/'progress.json',d)
            print(json.dumps({'progress':d},ensure_ascii=False),flush=True)
        robot_data=json.loads(Path(args.robot).read_text(encoding='utf-8')) if args.robot else {}
        robot=Robot(**robot_data.get('simulation_parameters',robot_data)).validate()
        result=run(field,cfg,robot=robot,progress=progress,cancel=lambda:(out/'cancel.flag').exists())
        save_result(result,out,plots=not args.no_plots,samples=not args.no_samples)
        print(json.dumps({'result':str(out),'metrics':result.metrics},ensure_ascii=False,allow_nan=False),flush=True)
        return 0
    except (KeyboardInterrupt,Cancelled):
        write_json(out/'status.json',{'schema':SCHEMA,'status':'cancelled','validation_passed':False});return 130
    except Exception as e:
        write_json(out/'status.json',{'schema':SCHEMA,'status':'failed','validation_passed':False,'error':str(e)})
        (out/'error.log').write_text(traceback.format_exc(),encoding='utf-8')
        print(json.dumps({'error':str(e)},ensure_ascii=False),file=sys.stderr,flush=True);return 2

if __name__=='__main__':raise SystemExit(main())
