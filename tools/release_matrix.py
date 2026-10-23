"""Reproducible release matrix, not a claim to cover every possible field.

Run from the repository: python tools/release_matrix.py --workers 4
Coarse grids exercise combinations; separate dense integration cases exercise
0.12 m² coverage. The selected and actual algorithms/fallbacks are recorded.
"""
from __future__ import annotations
from pathlib import Path
import sys,os
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
os.environ.setdefault('OPENBLAS_NUM_THREADS','1');os.environ.setdefault('MPLBACKEND','Agg')
import argparse,csv,json,time,traceback,itertools
from concurrent.futures import ProcessPoolExecutor,as_completed
from ccpp_mowing.models import Config,SOLVERS,INTER_SOLVERS,PLANNERS
from ccpp_mowing.maps import generate
from ccpp_mowing.pipeline import run,write_json


def case_run(case):
    t=time.perf_counter()
    try:
        field=generate(case['shape'],case['width'],case['height'],case['density'],case['distribution'],case['seed'])
        cfg=Config(grid_type=case['grid'],solver=case['intra'],cluster_solver=case['inter'],planner=case['planner'],
            partition=case['partition'],cell_area=case['cell_area'],cluster_size=16,cluster_min=3,seed=case['seed'],
            iterations=900,timeout=.3,prm_samples=300,ga_population=12,ga_generations=18,sa_iterations=250)
        r=run(field,cfg)
        return {**case,'outcome':'verified','centres':r.metrics['candidate_centres'],'serviced':r.metrics['serviced_centres'],
            'clusters':r.metrics['cluster_count'],'area_coverage':r.metrics['original_area_coverage_ratio'],
            'declared_coverage':r.metrics['declared_area_coverage_ratio'],'residual_m2':r.metrics['manual_handover_area_m2'],
            'connector_fallbacks':r.metrics['connector_fallbacks'],'ordering_substitutions':r.metrics['ordering_substitutions'],
            'actual_inter':r.metrics['actual_inter_solver'],'actual_calls':json.dumps(r.metrics['connector_actual_counts']),
            'numeric_solves':r.optimization['accepted_numeric_solves'],'length_m':r.metrics['path_length_m'],
            'simulation_seconds':r.metrics['trajectory_duration_s'],'compute_seconds':time.perf_counter()-t,
            'actual_obstacles':field.metadata['placed_obstacles'],'error':''}
    except Exception as e:
        return {**case,'outcome':'failed','compute_seconds':time.perf_counter()-t,'error':str(e),'traceback':traceback.format_exc()}


def cases():
    out=[];shapes=['rectangle','l_shape','trapezoid','notched','orchard'];dens=['low','medium','high']
    for i,(intra,inter,grid,planner) in enumerate(itertools.product(SOLVERS,INTER_SOLVERS,('square','triangle'),PLANNERS)):
        out.append({'id':f'alg_{i:04}','group':'algorithm_cross_product','intra':intra,'inter':inter,'grid':grid,'planner':planner,
            'partition':('bfs','kmeans')[i%2],'shape':shapes[(i//2)%5],'density':dens[(i//7)%3],
            'distribution':('uniform','clustered','rows')[(i//11)%3],'seed':7+i%3,'width':14.,'height':10.,'cell_area':1.})
    for j,(shape,density,distribution,grid) in enumerate(itertools.product(shapes,['none','low','medium','high'],['uniform','clustered','rows'],['square','triangle'])):
        out.append({'id':f'map_{j:04}','group':'map_distribution_cross_product','intra':SOLVERS[j%len(SOLVERS)],
            'inter':INTER_SOLVERS[(j//3)%len(INTER_SOLVERS)],'grid':grid,'planner':PLANNERS[(j//5)%len(PLANNERS)],
            'partition':('bfs','kmeans')[j%2],'shape':shape,'density':density,'distribution':distribution,
            'seed':11,'width':18.,'height':14.,'cell_area':.6})
    return out


def main():
    p=argparse.ArgumentParser();p.add_argument('--workers',type=int,default=4);p.add_argument('--limit',type=int)
    p.add_argument('--out',default=str(ROOT/'docs'/'qa'/'release_matrix'));a=p.parse_args()
    cs=cases()[:a.limit];rows=[];out=Path(a.out);out.mkdir(parents=True,exist_ok=True);start=time.perf_counter()
    with ProcessPoolExecutor(max_workers=a.workers) as pool:
        fs={pool.submit(case_run,c):c for c in cs}
        for future in as_completed(fs):
            row=future.result();rows.append(row)
            if len(rows)%30==0 or row['outcome']=='failed':print(f'{len(rows)}/{len(cs)} {row["id"]}: {row["outcome"]} {row.get("error","")}',flush=True)
    rows.sort(key=lambda r:r['id']);keys=sorted(set().union(*(r.keys() for r in rows))-{'traceback'})
    with (out/'matrix.csv').open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,keys,extrasaction='ignore');w.writeheader();w.writerows(rows)
    write_json(out/'failures.json',[r for r in rows if r['outcome']!='verified'])
    summary={'cases':len(rows),'verified':sum(r['outcome']=='verified' for r in rows),
             'full_centres':sum(r.get('serviced',-1)==r.get('centres',-2) for r in rows),
             'with_connector_fallback':sum(r.get('connector_fallbacks',0)>0 for r in rows),
             'with_ordering_substitution':sum(r.get('ordering_substitutions',0)>0 for r in rows),
             'elapsed_wall_seconds':time.perf_counter()-start,'workers':a.workers,
             'scope':'864 algorithm combinations + 120 map/density/distribution combinations on coarse integration grids; not all possible seeds or fine-grid cases'}
    write_json(out/'summary.json',summary);print(json.dumps(summary),flush=True)
    return 0 if summary['verified']==len(rows) else 1
if __name__=='__main__':raise SystemExit(main())
