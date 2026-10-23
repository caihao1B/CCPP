"""Reproduce the v2.1 integrated portfolio experiment (no hidden pivot fallback).
Run from the project root: python tools/build_comparison_suite.py --out outputs/comparison
Existing case folders are refused; inputs and ordered task IDs are retained.
"""
from pathlib import Path
from dataclasses import replace,asdict
import argparse,csv,hashlib,json,sys,time,math
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from shapely.geometry import Polygon,box,mapping
from ccpp_mowing.models import Config,Robot
from ccpp_mowing.geometry import FieldMap
from ccpp_mowing.pipeline import run,save_result,validate_saved,write_json
from ccpp_mowing.motion import evaluate_events
from ccpp_mowing.curve_motion import evaluate_curve_event
from ccpp_mowing.curves import wrap


def comparison_field():
    return FieldMap.from_geojson({'type':'FeatureCollection','coordinate_system':'local Cartesian metres',
      'metadata':{'synthetic':True,'purpose':'fixed irregular boundary + fixed obstacle; square/triangle x coverage ordering'},
      'features':[{'type':'Feature','properties':{'kind':'field'},'geometry':mapping(Polygon([(0,0),(12,0),(12,7),(10.5,9.5),(2,9.5),(0,7)]))},
                  {'type':'Feature','properties':{'kind':'obstacle'},'geometry':mapping(box(5.4,4.1,6.6,5.3))}]})


def continuity_audit(events,robot):
    pos_error=yaw_error=nonholo_error=0.;minimum_v=math.inf;min_track=math.inf;n=0
    for a,b in zip(events,events[1:]):
        sample=evaluate_events(events,[a['t1']-1e-6,a['t1']+1e-6],robot)
        pos_error=max(pos_error,float(np.linalg.norm(sample[0,1:3]-sample[1,1:3])))
        yaw_error=max(yaw_error,abs(wrap(sample[0,3]-sample[1,3])))
    for e in events:
        if 'curve' not in e:continue
        t=np.linspace(.01,.99,13)*e['duration'];h=1e-5
        a=evaluate_curve_event(e,t,robot)
        d=(evaluate_curve_event(e,t+h,robot)['pos']-evaluate_curve_event(e,t-h,robot)['pos'])/(2*h)
        nonholo_error=max(nonholo_error,float(np.max(abs(d-a['v'][:,None]*np.c_[np.cos(a['yaw']),np.sin(a['yaw'])]))))
        minimum_v=min(minimum_v,float(min(a['v'])))
        min_track=min(min_track,float(min(a['v']-robot.track_spacing_m/2*abs(a['omega']))));n+=len(t)
    result={'event_boundaries':max(0,len(events)-1),'moving_samples':n,'max_boundary_position_difference_m':pos_error,
       'max_boundary_heading_difference_rad':yaw_error,'max_forward_nonholonomic_residual_mps':nonholo_error,
       'minimum_sampled_forward_speed_mps':minimum_v,'minimum_sampled_equivalent_wheel_speed_mps':min_track,
       'pivot_events':sum(e['kind']=='TURN' for e in events),
       'notes':'Finite differences supplement (not replace) the independent continuous-bound certificate. Wheel speeds are model quantities.'}
    assert pos_error<1e-5 and yaw_error<1e-5 and nonholo_error<1e-4 and minimum_v>=-1e-9 and result['pivot_events']==0,result
    return result


def build(out,ids=None):
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    cfg=Config(trajectory_method='bezier_slsqp',solver='greedy',cell_area=.36,cluster_size=20,
        manual_setback_m=1.6,min_turn_radius_m=.18,navigation_resolution=.4,direct_visible=True,
        fallback_policy='strict',allow_partial=False,turn_policy='loops_only',curve_allow_pivot_fallback=False)
    rb=Robot(length_m=.65,width_m=.48,track_spacing_m=.30,work_speed_mps=.45,transit_speed_mps=.65,
        linear_acceleration_mps2=.30,linear_jerk_mps3=.60)
    field=comparison_field();cases=[]
    for grid in ('square','triangle'):
        for key,solver in [('greedy','greedy'),('mst','mst_euler'),('ga','ga')]:
            cases.append((f'{grid}_{key}',field,replace(cfg,grid_type=grid,solver=solver),rb,'grid_algorithm'))
    cases += [('square_ga_circular',field,replace(cfg,solver='ga',trajectory_method='circular_fillet'),rb,'geometry'),
              ('square_ga_conservative',field,replace(cfg,solver='ga',timing='conservative'),rb,'timing')]
    car=Config.from_dict(json.loads((ROOT/'configs/demo_04.json').read_text()))
    for radius in (.60,.90):
        cases.append((f'car_r{int(radius*100):03d}',FieldMap.load(ROOT/'maps/forward_open.geojson'),
          replace(car,min_turn_radius_m=radius,turn_policy='loops_only',fallback_policy='strict',allow_partial=False,curve_allow_pivot_fallback=False),Robot(),'radius'))
    summary=[]
    for key,field,cfg,rb,group in cases:
        if ids and key not in ids:continue
        folder=out/key
        if folder.exists():raise FileExistsError(f'Use a new output folder: {folder}')
        t=time.perf_counter()
        try:
            result=run(field,cfg,rb);save_result(result,folder,plots=False,samples=False)
            valid=validate_saved(folder)
            if not valid['valid']:raise ValueError(valid['errors'])
            audit=continuity_audit(result.events,rb)
            visits=[];counts={}
            for sequence,node in enumerate(result.order.order,1):
                counts[node]=counts.get(node,0)+1;p=result.grid.positions[node]
                visits.append(dict(sequence=sequence,node_id=int(node),visit_number=counts[node],revisit=counts[node]>1,x_m=p[0],y_m=p[1]))
            write_json(folder/'node_visit_order.json',visits)
            with (folder/'node_visit_order.csv').open('w',encoding='utf-8-sig',newline='') as f:
                writer=csv.DictWriter(f,fieldnames=list(visits[0]));writer.writeheader();writer.writerows(visits)
            write_json(folder/'continuity_audit.json',audit)
            order_sha=hashlib.sha256(json.dumps(result.order.order).encode()).hexdigest()
            info=dict(id=key,group=group,valid=True,elapsed_s=time.perf_counter()-t,config=cfg.to_dict(),robot=asdict(rb),
                      order_sha256=order_sha,planned_visits=len(visits),metrics=result.metrics,stages=result.stages,
                      heading_loops=result.optimization['heading_loops'],cross_row_loops=result.optimization['cross_row_loops'],audit=audit)
            write_json(folder/'comparison_case.json',info);summary.append(info)
            print(json.dumps({k:info[k] for k in ('id','valid','elapsed_s','planned_visits','heading_loops')},ensure_ascii=False),flush=True)
        except Exception as e:
            info=dict(id=key,group=group,valid=False,error=str(e));summary.append(info);print(json.dumps(info),flush=True)
    write_json(out/'summary.json',summary)
    return summary

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',default='outputs/comparison');p.add_argument('--ids',nargs='*');a=p.parse_args()
    records=build(a.out,a.ids);raise SystemExit(0 if all(r['valid'] for r in records) else 2)
