from pathlib import Path
import sys,os,json,time,traceback
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
os.environ.setdefault('OPENBLAS_NUM_THREADS','1');os.environ.setdefault('MPLBACKEND','Agg')
from concurrent.futures import ProcessPoolExecutor,as_completed
from ccpp_mowing.models import Config
from ccpp_mowing.geometry import FieldMap
from ccpp_mowing.maps import generate
from ccpp_mowing.pipeline import run,save_result,write_json


def work(c):
    try:
        field=FieldMap.load(ROOT/'maps'/'original_field.geojson') if c['map']=='original' else generate(c['shape'],c['width'],c['height'],c['density'],c['distribution'],c['seed'])
        cfg=Config(grid_type=c['grid'],cell_area=c['cell_area'],cluster_size=c.get('cluster_size',64),
            solver=c['intra'],cluster_solver=c['inter'],planner=c['planner'],partition=c.get('partition','bfs'),seed=c['seed'],
            navigation_resolution=.5,iterations=1800,timeout=.5,prm_samples=900)
        start=time.perf_counter();r=run(field,cfg)
        out=ROOT/'docs'/'qa'/'dense_integration'/c['id'];out.mkdir(parents=True,exist_ok=True)
        write_json(out/'metrics.json',r.metrics);write_json(out/'validation.json',r.validation);write_json(out/'config.json',cfg.to_dict())
        from ccpp_mowing.visualization import save_plots
        if c.get('plots',False):save_plots(r,out)
        return {**c,'verified':True,'seconds':time.perf_counter()-start,'metrics':r.metrics,'validation':r.validation}
    except Exception as e:return {**c,'verified':False,'error':str(e),'traceback':traceback.format_exc()}


def main():
    cases=[]
    for i,grid in enumerate(['square','triangle']):
        for j,intra in enumerate(['serpentine','mst_euler','ga','christofides','nn2opt','exact']):
            cases.append(dict(id=f'original_{grid}_{intra}',map='original',grid=grid,intra=intra,
                inter=['nn2opt','ga','sa','christofides','nearest','exact'][j],planner=['astar','prm','rrt','rrt_connect','rrt_star','astar'][j],
                cell_area=.12,cluster_size=64,seed=7,plots=intra=='serpentine'))
        for k,(shape,dist) in enumerate([('notched','clustered'),('orchard','rows')]):
            cases.append(dict(id=f'dense_{shape}_{grid}',map='generated',shape=shape,width=24,height=18,density='high',distribution=dist,
                grid=grid,intra='serpentine',inter='nn2opt',planner='prm',cell_area=.12,seed=19,plots=True))
    cases += [dict(id='large_024ha_square',map='generated',shape='rectangle',width=60,height=40,density='medium',distribution='uniform',grid='square',
                  intra='serpentine',inter='nn2opt',planner='astar',cell_area=.12,cluster_size=120,seed=7,plots=True),
              dict(id='scale_1ha_triangle_coarse',map='generated',shape='rectangle',width=100,height=100,density='low',distribution='rows',grid='triangle',
                  intra='serpentine',inter='nearest',planner='astar',cell_area=1.,cluster_size=120,seed=7,plots=True)]
    rows=[];root=ROOT/'docs'/'qa'/'dense_integration';root.mkdir(exist_ok=True)
    with ProcessPoolExecutor(max_workers=2) as pool:
        fs=[pool.submit(work,c) for c in cases]
        for f in as_completed(fs):
            r=f.result();rows.append(r);print(r['id'],r['verified'],r.get('error',''),flush=True)
            write_json(root/'summary.json',rows)
    return 0 if all(r['verified'] for r in rows) else 1
if __name__=='__main__':raise SystemExit(main())
