"""One execution path shared by CLI, GUI, tests and reproducible experiments."""
from __future__ import annotations
from collections import Counter
from dataclasses import dataclass, asdict
import csv, hashlib, json, math, platform, sys, time
from pathlib import Path
import networkx as nx
import numpy as np
from .models import Config, Robot, PlanningError, Cancelled
from .geometry import World, FieldMap, save_geojson, parts
from .grids import build_grid
from .navigation import Navigator
from .ordering import build_order
from .motion import build_events, sample_events, COLUMNS
from .validation import validate

SCHEMA='ccpp.mowing.release.v2'
SUPPORTED_SCHEMAS=(SCHEMA,'ccpp.mowing.release.v1')


def write_json(path,data):
    Path(path).write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')

@dataclass
class RunResult:
    config: Config
    robot: Robot
    world: World
    grid: object
    order: object
    events: list
    legs: list
    required: dict
    skipped: list
    queries: list
    validation: dict
    areas: dict
    metrics: dict
    optimization: dict
    stages: dict


def run(field:FieldMap,cfg:Config|None=None,robot:Robot|None=None,progress=None,cancel=None):
    cfg=(cfg or Config()).validate();robot=(robot or Robot()).validate()
    if abs(cfg.tool_radius-robot.cutter_radius)>1e-12:
        raise ValueError('Config tool_radius must match the selected robot cutter; set tool_radius to half the selected cutting width')
    progress=progress or (lambda stage,fraction,text:None)
    cancel=cancel or (lambda:False)
    t0=time.perf_counter();stages={}
    def check():
        if cancel():raise Cancelled('Experiment cancelled; no complete result is certified')
    def tick(stage,fraction,text):check();progress(stage,float(fraction),text)
    tick('map',.02,'Metric field and the selected full vehicle footprint')
    world=World(field,robot,cfg.manual_setback_m)
    tick('grid',.05,'Regular square/triangle service cells')
    t=time.perf_counter();grid=build_grid(world,cfg);stages['grid_s']=time.perf_counter()-t
    candidate_count=len(grid.positions)
    tick('navigation',.12,f'{candidate_count} safe centre candidates; building a shared transit graph')
    t=time.perf_counter();nav=Navigator(world,cfg,check);keep,skipped=nav.restrict_tasks(grid)
    stages['navigation_s']=time.perf_counter()-t
    if not keep:raise PlanningError('No task centre is attachable to the navigation graph; manual work required')
    if skipped and not cfg.allow_partial:raise PlanningError(f'{len(skipped)} centres are unreachable under the declared model/resolution')
    grid.graph=grid.graph.subgraph(keep).copy();grid.positions={u:grid.positions[u] for u in keep}
    reasons={s['id']:s['reason'] for s in skipped}
    for c in grid.cells:
        if c.id in reasons:c.traversable=False;c.reason=reasons[c.id]
    grid.metadata.update(candidate_centres=candidate_count,reachable_centres=len(keep),unreachable_centres=len(skipped))
    tick('ordering',.23,'Connected partitions, inter-cluster tour, gates and local coverage')
    t=time.perf_counter();order=build_order(grid,cfg,nav,check);stages['ordering_s']=time.perf_counter()-t
    labels={n:i for i,g in enumerate(order.clusters) for n in g}
    current=order.order[0];start=grid.positions[current];completed={current};legs=[]
    t=time.perf_counter()
    for i,target in enumerate(order.order[1:],1):
        check()
        if i%50==1:tick('connections',.30+.26*i/max(1,len(order.order)),f'{i}/{len(order.order)-1} task connections')
        if target==current:continue
        new=target not in completed
        try:path=nav.route(grid.positions[current],grid.positions[target])
        except Cancelled:
            raise
        except PlanningError as e:
            if not cfg.allow_partial:raise
            skipped.append({'id':target,'center':grid.positions[target],'reason':'connection_budget_or_discretization_failure','detail':str(e)})
            continue
        for j,(a,b) in enumerate(zip(path,path[1:])):
            kind='WORK' if new and j==len(path)-2 else 'TRANSIT'
            reason='first_service_arrival' if kind=='WORK' else ('cluster_transition' if labels[current]!=labels[target] else 'safe_revisit_or_detour')
            legs.append({'p0':a,'p1':b,'kind':kind,'cluster':labels[target],'reason':reason,
                         'tasks':[target] if j==len(path)-2 else []})
        completed.add(target);current=target
    if cfg.return_to_start and math.dist(grid.positions[current],start)>1e-7:
        # A failed requested return is not silently declared a completed closed tour.
        raise PlanningError('Could not return to start within the selected connection policy')
    required={u:grid.positions[u] for u in completed}
    # Any skipped target later reached is not still labelled unserviced.
    skipped=[s for s in skipped if s['id'] not in completed]
    stages['connections_s']=time.perf_counter()-t
    tick('trajectory',.59,'Finite-rate track motion and constrained numerical time profiles')
    t=time.perf_counter()
    if cfg.trajectory_method=='stop_turn_go':
        events,opt=build_events(start,legs,robot,cfg,check)
    else:
        from .curve_motion import build_curve_events
        events,opt=build_curve_events(start,legs,world,robot,cfg,check)
    stages['trajectory_s']=time.perf_counter()-t
    tick('validation',.76,'Independent swept footprint, motion extrema and service-area validation')
    t=time.perf_counter();report,areas,metrics=validate(world,robot,events,required,cfg.return_to_start,cfg=cfg)
    stages['validation_s']=time.perf_counter()-t
    if not report['valid']:raise PlanningError(f'Independent validation failed: {report["errors"][:4]}')
    kinds=Counter(e['kind'] for e in events);actual=Counter(q['actual'] for q in nav.logs)
    unfulfilled=candidate_count-len(required)
    metrics.update(schema=SCHEMA,robot_reference=robot.metadata()['reference'],grid=cfg.grid_type,intra_solver=cfg.solver,
        requested_inter_solver=cfg.cluster_solver,actual_inter_solver=order.stats['actual_inter_solver'],
        partition=cfg.partition,planner=cfg.planner,cell_area_m2=grid.area,
        candidate_centres=candidate_count,serviced_centres=len(required),unserviced_candidate_centres=unfulfilled,
        centre_completion_ratio=len(required)/candidate_count,cluster_count=len(order.clusters),
        event_counts=dict(kinds),trajectory_duration_s=events[-1]['t1'],
        time_optimization_reduction_ratio=opt['time_reduction_ratio'],
        connector_actual_counts=dict(actual),connector_fallbacks=sum(q['fallback'] is not None for q in nav.logs),
        ordering_substitutions=len(order.stats['warnings']),navigation_nodes=len(nav.road.graph),
        certified_narrow_straight_bridges=len(nav.bridge_log),initially_skipped_centres=len(reasons),
        return_to_start_verified=cfg.return_to_start,validation_passed=True,
        status='verified_with_manual_residual' if metrics['manual_handover_area_m2']>1e-6 or unfulfilled else 'verified_complete',
        compute_seconds=time.perf_counter()-t0,
        interpretation='Centre obligations and original/declared area obligations are separate. A valid partial result is not 100% coverage.')
    metrics.update(trajectory_method=cfg.trajectory_method,motion_model=cfg.motion_model,
        minimum_turn_radius_m=cfg.min_turn_radius_m,
        service_rule='centre_exact' if cfg.trajectory_method=='stop_turn_go' else cfg.curve_service_rule,
        curve_primitive_counts=opt.get('curve_counts',{}),pivot_fallbacks=opt.get('pivot_fallbacks',0),
        raw_polyline_length_m=sum(math.dist(l['p0'],l['p1']) for l in legs),
        robot_motion_interpretation='ideal reference trajectory replay; no feedback controller, slip or hardware validation')
    stages['total_s']=time.perf_counter()-t0
    tick('complete',1.,f'Validated {len(required)}/{candidate_count} centres; manual residual {metrics["manual_handover_area_m2"]:.2f} m²')
    return RunResult(cfg,robot,world,grid,order,events,legs,required,skipped,nav.logs,report,areas,metrics,opt,stages)


def save_result(result:RunResult,out,plots=True,samples=True):
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    write_json(out/'status.json',{'schema':SCHEMA,'status':'writing'})
    write_json(out/'config.json',result.config.to_dict());write_json(out/'robot.json',result.robot.metadata())
    write_json(out/'map.geojson',result.world.map.original_geojson)
    write_json(out/'metrics.json',result.metrics);write_json(out/'validation.json',result.validation)
    write_json(out/'stages.json',result.stages);write_json(out/'events.json',result.events)
    write_json(out/'raw_legs.json',result.legs)
    write_json(out/'optimization.json',result.optimization);write_json(out/'queries.json',result.queries)
    write_json(out/'ordering.json',asdict(result.order));write_json(out/'skipped_tasks.json',result.skipped)
    write_json(out/'required_centres.json',{str(k):v for k,v in result.required.items()})
    write_json(out/'grid.geojson',result.grid.to_geojson())
    save_geojson(out/'coverage.geojson',[(name,g,{}) for name,g in result.areas.items()])
    save_geojson(out/'task_masks.geojson',[(k,g,{}) for k,g in [('original',result.world.original_task),('declared',result.world.target),('predeclared_manual',result.world.manual_predeclared)]])
    with (out/'manual_handover.csv').open('w',newline='',encoding='utf-8-sig') as f:
        writer=csv.writer(f);writer.writerow(['id','reason','area_m2','xmin','ymin','xmax','ymax'])
        i=0
        for reason,key in [('predeclared_boundary_or_obstacle_finishing','predeclared_remaining'),('declared_task_not_covered','declared_remaining')]:
            for p in parts(result.areas[key]):
                writer.writerow([i,reason,p.area,*p.bounds]);i+=1
    if samples:
        values,dt=sample_events(result.events,result.robot,result.config)
        np.savetxt(out/'trajectory.csv',values,delimiter=',',header=','.join(COLUMNS),comments='',fmt='%.8f')
        write_json(out/'sampling.json',{'requested_dt_s':result.config.sample_dt_s,'actual_dt_s':dt,
                   'samples':len(values),'warning':'CSV is a visualization/export sample. Whole-primitive analytic validation does not depend on this interval.'})
    if result.config.trajectory_method!='stop_turn_go':
        from .curve_motion import curve_diagnostics,DIAGNOSTIC_COLUMNS
        from .curves import flatten,at,curvature
        records=[]
        for i,e in enumerate(result.events):
            if 'curve' not in e:continue
            g=e['curve'];us,xy=flatten(g,result.config.curve_tolerance_m,result.config.curve_step_m)
            yaw=at(g,us)[-1];kap=curvature(g,us)
            for u,p,y,k in zip(us,xy,yaw,kap):records.append([*p,y,k,i,int(e['tool_on']),u])
        np.savetxt(out/'curve_path.csv',np.asarray(records).reshape(-1,7),delimiter=',',
            header='x,y,yaw,curvature_inv_m,event_id,tool_on,u',comments='',fmt='%.9f')
        if samples:
            diagnostics=curve_diagnostics(result.events,values[:,0],result.robot,result.config)
            np.savetxt(out/'curve_diagnostics.csv',diagnostics,delimiter=',',
                header=','.join(DIAGNOSTIC_COLUMNS),comments='',fmt='%.9f')
            np.savetxt(out/'trajectory_reference.csv',np.column_stack([values,diagnostics[:,1:7]]),delimiter=',',
                header=','.join(COLUMNS+DIAGNOSTIC_COLUMNS[1:7]),comments='',fmt='%.9f')
    import scipy,shapely,networkx,matplotlib
    write_json(out/'environment.json',{'python':sys.version,'platform':platform.platform(),'numpy':np.__version__,
       'scipy':scipy.__version__,'shapely':shapely.__version__,'networkx':networkx.__version__,'matplotlib':matplotlib.__version__})
    if plots:
        from .visualization import save_plots
        save_plots(result,out)
    # Status is written last. GUI/loaders reject an incomplete directory.
    hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in out.iterdir() if p.is_file() and p.name not in ('status.json','checksums.json')}
    write_json(out/'checksums.json',hashes)
    write_json(out/'status.json',{'schema':SCHEMA,'status':result.metrics['status'],'validation_passed':True})
    return out


def validate_saved(out):
    """Recompute from events and physical geometry; checksums are not proof."""
    out=Path(out);status=json.loads((out/'status.json').read_text())
    if status.get('schema') not in SUPPORTED_SCHEMAS or status.get('status') not in ('verified_complete','verified_with_manual_residual'):
        raise PlanningError('Incomplete/failed/cancelled/old-schema output cannot be loaded as a verified experiment')
    cfg=Config.from_dict(json.loads((out/'config.json').read_text()));robot=Robot(**json.loads((out/'robot.json').read_text())['simulation_parameters']).validate()
    world=World(FieldMap.load(out/'map.geojson'),robot,cfg.manual_setback_m)
    ev=json.loads((out/'events.json').read_text());req=json.loads((out/'required_centres.json').read_text())
    report,_,metrics=validate(world,robot,ev,req,cfg.return_to_start,cfg=cfg)
    claimed=json.loads((out/'metrics.json').read_text())
    regenerated=build_grid(world,cfg)
    all_centres={str(k):p for k,p in regenerated.positions.items()}
    skipped=json.loads((out/'skipped_tasks.json').read_text())
    skip_ids={str(s['id']) for s in skipped}
    required_ids=set(req)
    if (required_ids & skip_ids or required_ids | skip_ids != set(all_centres)
            or int(claimed['candidate_centres'])!=len(all_centres)
            or int(claimed['serviced_centres'])!=len(req)):
        report['valid']=False;report['errors'].append('Task ledger differs from regenerated input grid')
    for k,p in req.items():
        if k not in all_centres or math.dist(p,all_centres[k])>1e-8:
            report['valid']=False;report['errors'].append('Required centre was relocated')
    if cfg.start_id is not None and (str(cfg.start_id) not in req or math.dist(ev[0]['p0'],req[str(cfg.start_id)])>1e-8):
        report['valid']=False;report['errors'].append('Explicit start was changed')
    if not cfg.allow_partial and skip_ids:
        report['valid']=False;report['errors'].append('Strict task policy does not allow skipped targets')
    if len(skipped) != len(skip_ids):
        report['valid']=False;report['errors'].append('Duplicate skipped target entries')
    for item in skipped:
        key=str(item['id'])
        if (key not in all_centres or not item.get('reason') or
                math.dist(item.get('center', (float('inf'), float('inf'))), all_centres.get(key, (0,0)))>1e-8):
            report['valid']=False;report['errors'].append('Invalid skipped-target coordinate/reason')
    # The GUI displays these numbers; stale or edited summaries must not be
    # certified simply because the underlying geometry is still feasible.
    recomputed = dict(metrics)
    recomputed.update(trajectory_duration_s=ev[-1]['t1'] if ev else 0.,
        event_counts=dict(Counter(e['kind'] for e in ev)),
        centre_completion_ratio=len(req)/len(all_centres) if all_centres else 0.,
        unserviced_candidate_centres=len(all_centres)-len(req))
    if cfg.trajectory_method != 'stop_turn_go':
        recomputed.update(curve_primitive_counts=dict(Counter(e['curve']['type'] for e in ev if 'curve' in e)),
            pivot_fallbacks=sum(e['kind']=='TURN' for e in ev))
    for name, expected in recomputed.items():
        if name not in claimed:
            # Legacy optional diagnostics may be absent. Core metrics must exist.
            if name in ('trajectory_duration_s','declared_area_coverage_ratio','event_counts'):
                report['valid']=False;report['errors'].append(f'Missing saved metric: {name}')
            continue
        actual=claimed[name]
        if isinstance(expected, (int,float)) and not isinstance(expected,bool):
            okay=(isinstance(actual,(int,float)) and not isinstance(actual,bool) and
                  math.isfinite(actual) and abs(actual-expected)<=1e-5*max(1.,abs(expected)))
        elif expected is None or isinstance(expected,dict):
            okay=(actual==expected)
        else:
            continue  # Human-readable interpretation text is not a numeric claim.
        if not okay:
            report['valid']=False;report['errors'].append(f'Recomputed {name} disagrees with saved metric')
    metrics.update(trajectory_duration_s=recomputed['trajectory_duration_s'])
    report['recomputed_metrics']=metrics
    return report
