"""Exercise each PRIMARY planner on blocked straight-line queries; no recovery."""
from pathlib import Path
import sys, time
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from shapely.geometry import Point
from ccpp_mowing.models import Robot
from ccpp_mowing.maps import generate
from ccpp_mowing.geometry import World,path_length
from ccpp_mowing.search import GridGraph,PRM,rrt,rrt_connect
from ccpp_mowing.pipeline import write_json

def main():
    rows=[]
    for seed in (0,5,19):
        for planner in ('astar','dijkstra','rrt','rrt_connect','rrt_star','prm'):
            m=generate(width=12,height=10,density='none');m.obstacles=[Point(6,5).buffer(1.1)]
            w=World(m,Robot());p,q=(2.,5.),(10.,5.);t=time.perf_counter()
            assert not w.segment_free(p,q)
            if planner in ('astar','dijkstra'):r=GridGraph(w,.45).query(p,q,heuristic=planner=='astar')
            elif planner=='prm':r=PRM(w,500,16,seed).query(p,q)
            elif planner=='rrt_connect':r=rrt_connect(w,p,q,seed=seed,iterations=4000,timeout=4.)
            else:r=rrt(w,p,q,seed=seed,iterations=2200,timeout=4.,star=planner=='rrt_star')
            ok=bool(r.success and len(r.path)>2 and w.path_free(r.path))
            rows.append({'planner':planner,'seed':seed,'primary_success':ok,'recovery_used':False,
                         'seconds':time.perf_counter()-t,'length_m':path_length(r.path) if r.path else None,'reason':r.reason})
    write_json(ROOT/'docs/qa/forced_detours.json',{'cases':len(rows),'passed':sum(r['primary_success'] for r in rows),'results':rows})
    print(f"Primary-only blocked-route cases: {sum(r['primary_success'] for r in rows)}/{len(rows)}")
    return 0 if all(r['primary_success'] for r in rows) else 1
if __name__=='__main__':raise SystemExit(main())
