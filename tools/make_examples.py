"""Generate checked-in examples from the public pipeline, not hand-drawn paths."""
from pathlib import Path
import sys
from dataclasses import replace
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from ccpp_mowing.models import Config
from ccpp_mowing.maps import generate
from ccpp_mowing.pipeline import run, save_result

if __name__ == '__main__':
    field = generate('rectangle',18.,14.,'medium','uniform',7)
    for grid in ('square', 'triangle'):
        cfg = replace(Config(),grid_type=grid)
        result = run(field,cfg)
        out = ROOT/'examples'/f'{grid}_default'
        save_result(result,out)
        print(grid,result.metrics,flush=True)
