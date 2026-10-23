"""Deterministic independent audit, supplementing the project's unit tests.
Runs graph-optimality cross-checks, 24 whole-pipeline combinations, dense
physical replay checks and expected failures. No external credentials required.
"""
import sys,json,math,time
from pathlib import Path
from dataclasses import replace
import numpy as np
import networkx as nx
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from ccpp_mowing.models import Config,Robot,SOLVERS,PlanningError
from ccpp_mowing.maps import generate
from ccpp_mowing.pipeline import run,write_json,validate_saved
from ccpp_mowing.search import astar_graph,GridGraph,PRM,rrt,rrt_connect
from ccpp_mowing.motion import evaluate_events,Profile
from ccpp_mowing.curve_motion import evaluate_curve_event
from ccpp_mowing.geometry import World,FieldMap,rectangle,path_length
from shapely.geometry import Point


def audit(out):
    out=Path(out);out.mkdir(parents=True,exist_ok=True);rows=[]
    def record(name,fn):
        t=time.perf_counter()
        try:extra=fn();row={'name':name,'passed':True,'seconds':time.perf_counter()-t,**(extra or {})}
        except Exception as e:row={'name':name,'passed':False,'seconds':time.perf_counter()-t,'error':repr(e)}
        rows.append(row);print(json.dumps(row),flush=True);write_json(out/'extended.json',rows)
    def graph_check():
        for seed in range(100):
            rng=np.random.default_rng(seed);pos={i:tuple(rng.uniform(-5,5,2)) for i in range(24)};G=nx.Graph()
            for i in range(23):G.add_edge(i,i+1,weight=math.dist(pos[i],pos[i+1])*(1+rng.random()))
            for i in range(24):
                for j in range(i+2,24):
                    if rng.random()<.15:G.add_edge(i,j,weight=math.dist(pos[i],pos[j])*(1+rng.random()))
            def neighbors(u):return [(v,d['weight']) for v,d in G[u].items()]
            path,cost,expanded=astar_graph(neighbors,0,23,pos)
            exact=nx.dijkstra_path_length(G,0,23,weight='weight')
            assert abs(cost-exact)<1e-8,(seed,cost,exact)
            assert path[0]==0 and path[-1]==23
        return {'weighted_graphs':100,'cost_model':'edge cost >= Euclidean distance; admissible heuristic'}
    record('astar_vs_networkx_dijkstra',graph_check)
    for grid in ('square','triangle'):
        for solver in SOLVERS:
            def trial(grid=grid,solver=solver):
                cfg=Config(grid_type=grid,solver=solver,cluster_size=9,cluster_min=2,
                    cell_area=.64,trajectory_method='bezier_slsqp',min_turn_radius_m=.1,
                    allow_partial=False,fallback_policy='strict',ga_generations=12,sa_iterations=60)
                r=run(generate(width=8,height=6,density='none'),cfg)
                assert r.validation['valid'] and r.metrics['centre_completion_ratio']==1
                assert set(r.order.order)==set(r.required)
                return {'targets':len(r.required),'curves':r.metrics['curve_primitive_counts'],'pivots':r.metrics['pivot_fallbacks']}
            record('pipeline_'+grid+'_'+solver,trial)
    for seed in (0,5,19):
        for planner in ('astar','dijkstra','rrt','rrt_connect','rrt_star','prm'):
            def detour(seed=seed,planner=planner):
                field=generate(width=12,height=10,density='none');field.obstacles=[Point(6,5).buffer(1.1)]
                world=World(field,Robot());p,q=(2.,5.),(10.,5.);assert not world.segment_free(p,q)
                if planner in ('astar','dijkstra'):r=GridGraph(world,.45).query(p,q,heuristic=planner=='astar')
                elif planner=='prm':r=PRM(world,500,16,seed).query(p,q)
                elif planner=='rrt_connect':r=rrt_connect(world,p,q,seed=seed,iterations=4000,timeout=4.)
                else:r=rrt(world,p,q,seed=seed,iterations=2200,timeout=4.,star=planner=='rrt_star')
                assert r.success and len(r.path)>2 and world.path_free(r.path),r.reason
                return {'primary_only':True,'length_m':path_length(r.path),'stats':r.stats}
            record(f'blocked_detour_{planner}_seed{seed}',detour)
    for no in range(1,8):
        def replay(no=no):
            folder=ROOT/f'examples/demo_{no:02d}';read=lambda n:json.loads((folder/n).read_text())
            cfg=Config.from_dict(read('config.json'));rb=Robot(**read('robot.json')['simulation_parameters'])
            world=World(FieldMap.load(folder/'map.geojson'),rb,cfg.manual_setback_m);events=read('events.json')
            checked=0;max_pos_derivative=0.;max_yaw_derivative=0.;min_clearance=math.inf;maximum_lateral=0.
            for e in events:
                if 'curve' not in e:continue
                ts=e['duration']*np.linspace(.001,.999,41)
                val=evaluate_curve_event(e,ts,rb);h=1e-5
                plus=evaluate_curve_event(e,ts+h,rb);minus=evaluate_curve_event(e,ts-h,rb)
                dp=(plus['pos']-minus['pos'])/(2*h)
                expected=val['v'][:,None]*np.column_stack([np.cos(val['yaw']),np.sin(val['yaw'])])
                err=float(abs(dp-expected).max());max_pos_derivative=max(err,max_pos_derivative)
                dy=(plus['yaw']-minus['yaw']+math.pi)%(2*math.pi)-math.pi
                err_y=float(abs(dy/(2*h)-val['omega']).max());max_yaw_derivative=max(err_y,max_yaw_derivative)
                assert err<1e-5 and err_y<1e-5
                assert val['v'].min()>-1e-8
                maximum_lateral=max(maximum_lateral,float(abs(val['v']*val['omega']).max()))
                assert maximum_lateral<=cfg.lateral_acceleration_mps2+1e-6
                for pos,yaw in zip(val['pos'],val['yaw']):
                    body=rectangle(pos,yaw,rb.length_m,rb.width_m)
                    assert world.free.covers(body.buffer(rb.clearance_m,quad_segs=32))
                    min_clearance=min(min_clearance,body.distance(world.free.boundary));checked+=1
            rep=validate_saved(folder);assert rep['valid'],rep['errors']
            return {'sampled_full_body_poses':checked,'max_position_derivative_error':max_pos_derivative,
                'max_heading_derivative_error':max_yaw_derivative,'min_sampled_body_clearance_m':min_clearance if checked else None,
                'max_sampled_lateral_acceleration_mps2':maximum_lateral,
                'note':'additional dense sampled cross-check, not a replacement for continuous certificates'}
        record(f'independent_replay_demo_{no:02d}',replay)
    write_json(out/'extended_summary.json',{'passed':sum(r['passed'] for r in rows),'total':len(rows),'results':rows})
    return all(r['passed'] for r in rows)

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--out',default='outputs/audit');a=p.parse_args()
    raise SystemExit(0 if audit(a.out) else 2)
