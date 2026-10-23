"""Reproducible research cases for portfolio, not manufacturer performance claims.
Run: python tools/build_portfolio_cases.py --out outputs/portfolio
The same API as the GUI is used. Inputs, full node order and evidence are saved.
"""
from pathlib import Path
import argparse,sys,json,math,csv,time
from dataclasses import replace,asdict
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from shapely.geometry import box,mapping
from ccpp_mowing.models import Config,Robot
from ccpp_mowing.geometry import FieldMap
from ccpp_mowing.pipeline import run,save_result,validate_saved,write_json


def obstacle_map():
    return FieldMap.from_geojson({'type':'FeatureCollection','coordinate_system':'local Cartesian metres',
        'metadata':{'synthetic':True,'purpose':'visible A-star detour + node order; not real farm'},
        'features':[{'type':'Feature','properties':{'kind':'field'},'geometry':mapping(box(0,0,10,8))},
                    {'type':'Feature','properties':{'kind':'obstacle'},'geometry':mapping(box(4.1,2.6,5.9,5.4))}]})


def build(out,selected=None):
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    base=Config(trajectory_method='bezier_slsqp',solver='greedy',cell_area=.36,
        cluster_size=20,manual_setback_m=.8,min_turn_radius_m=.12,
        navigation_resolution=.4,direct_visible=False,shortcut_transit=True,
        fallback_policy='strict',allow_partial=False)
    compact=Robot(length_m=.65,width_m=.48,track_spacing_m=.42,work_speed_mps=.45,
        transit_speed_mps=.65,linear_acceleration_mps2=.3,linear_jerk_mps3=.6)
    car=Config.from_dict(json.loads((ROOT/'configs/demo_04.json').read_text()))
    cases=[
      ('compact_greedy_astar','Compact differential | Greedy + A* | optimized Bezier',obstacle_map(),base,compact),
      ('mower_mst_astar','Mower differential | MST-Euler + A* | circular fillet',obstacle_map(),
        replace(base,solver='mst_euler',trajectory_method='circular_fillet',min_turn_radius_m=.1),Robot()),
      ('compact_mst_astar','Same compact vehicle | MST-Euler + A* | optimized Bezier',obstacle_map(),
        replace(base,solver='mst_euler'),compact),
      ('obstacle_astar_bezier','Obstacle field | actual blocked A* query | optimized Bezier',obstacle_map(),
        replace(base,solver='serpentine',partition='none',max_metric_nodes=500,direct_visible=True,min_turn_radius_m=.1),Robot()),
      ('compact_ga_astar','Compact differential | GA + A* | optimized Bezier',obstacle_map(),
        replace(base,solver='ga'),compact),
      ('compact_greedy_conservative','Same geometry | conservative time scaling',obstacle_map(),
        replace(base,timing='conservative'),compact),
      ('forward_dubins_r060','Forward-only research car | Dubins | R >= 0.60 m',FieldMap.load(ROOT/'maps/forward_open.geojson'),
        replace(car,fallback_policy='strict',allow_partial=False),Robot()),
      ('forward_dubins_r090','Same forward-only research car | Dubins | R >= 0.90 m',FieldMap.load(ROOT/'maps/forward_open.geojson'),
        replace(car,min_turn_radius_m=.9,fallback_policy='strict',allow_partial=False),Robot()),
    ]
    records=[]
    for key,title,field,cfg,robot in cases:
        if selected and key not in selected:continue
        folder=out/key
        if folder.exists():raise FileExistsError(f'Refusing to replace {folder}; choose a new output directory')
        t=time.perf_counter()
        try:
            result=run(field,cfg,robot)
            save_result(result,folder,plots=False,samples=True)
            report=validate_saved(folder)
            assert report['valid'],report['errors']
            visits={};rows=[]
            for idx,n in enumerate(result.order.order,1):
                visits[n]=visits.get(n,0)+1
                rows.append({'sequence':idx,'node_id':int(n),'visit_number':visits[n],
                             'revisit':visits[n]>1,'x_m':result.grid.positions[n][0],
                             'y_m':result.grid.positions[n][1],
                             'meaning':'planned task visit, not first cutter contact'})
            with (folder/'node_visit_order.csv').open('w',newline='',encoding='utf-8-sig') as f:
                w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
            write_json(folder/'node_visit_order.json',rows)
            blocked=[q for q in result.queries if q['actual']=='astar' and not result.world.segment_free(q['start'],q['goal'])]
            info={'id':key,'title':title,'valid':True,'seconds':time.perf_counter()-t,
                  'actual_blocked_astar_queries':len(blocked),'metrics':result.metrics,
                  'robot':asdict(robot),'config':cfg.to_dict(),
                  'validation':report,'planned_visits':len(rows)}
            write_json(folder/'portfolio_case.json',info)
            records.append(info)
            print(json.dumps({k:info[k] for k in ('id','valid','seconds','actual_blocked_astar_queries','planned_visits')},ensure_ascii=False),flush=True)
        except Exception as e:
            records.append({'id':key,'valid':False,'error':str(e),'seconds':time.perf_counter()-t})
            print(json.dumps(records[-1]),flush=True)
    write_json(out/'portfolio_cases.json',records)
    return records

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',default='outputs/portfolio');p.add_argument('--ids',nargs='*');a=p.parse_args()
    results=build(a.out,a.ids)
    raise SystemExit(0 if all(r['valid'] for r in results) else 2)
