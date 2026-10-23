"""Rebuild reproducible v2 presets with the public CLI and independent validation.

Examples:
  python tools/build_curve_demos.py --ids 03 04
  python tools/build_curve_demos.py --ids all --out outputs/rebuilt --gifs
Existing output is refused unless --overwrite is explicitly supplied.
"""
from __future__ import annotations
import argparse,json,subprocess,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--ids',nargs='+',default=['03','04'])
    p.add_argument('--out',type=Path,default=ROOT/'outputs'/'rebuilt_demos')
    p.add_argument('--overwrite',action='store_true')
    p.add_argument('--gifs',action='store_true')
    a=p.parse_args(argv)
    presets=json.loads((ROOT/'demos'/'presets.json').read_text(encoding='utf-8'))
    valid={x['id'] for x in presets}
    if 'all' not in a.ids and not set(a.ids)<=valid:p.error('Unknown demo id; use 01..07 or all.')
    selected=presets if 'all' in a.ids else [x for x in presets if x['id'] in a.ids]
    rows=[];out=a.out.resolve();out.mkdir(parents=True,exist_ok=True)
    for item in selected:
        folder=out/('demo_'+item['id']);start=time.perf_counter()
        cmd=[sys.executable,'-m','ccpp_mowing','plan','--config',str(ROOT/item['config']),
             '--map',str(ROOT/item['map']),'--out',str(folder)]
        if a.overwrite:cmd.append('--overwrite')
        subprocess.run(cmd,cwd=ROOT,check=True)
        proc=subprocess.run([sys.executable,'-m','ccpp_mowing','validate',str(folder)],cwd=ROOT,check=True,capture_output=True,text=True,encoding='utf-8')
        verified=json.loads(proc.stdout)
        if not verified['valid']:raise RuntimeError('Independent validation did not pass')
        if a.gifs:
            subprocess.run([sys.executable,'-m','ccpp_mowing','render',str(folder),'--out',str(folder/'first_curve.gif')],cwd=ROOT,check=True)
        rows.append({'id':item['id'],'folder':str(folder),'valid':verified['valid'],'elapsed_seconds':time.perf_counter()-start})
    (out/'rebuild_report.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2),encoding='utf-8')
    print('Completed:',len(rows),'presets. This is reference replay, not measured tracking.')
    return 0
if __name__=='__main__':raise SystemExit(main())
