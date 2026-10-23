"""Read saved results for display without unpickling or re-running the planner."""
from pathlib import Path
import json
from types import SimpleNamespace
from shapely.geometry import shape
from .models import Config, Robot, PlanningError
from .geometry import FieldMap, World
from .pipeline import SCHEMA,SUPPORTED_SCHEMAS,validate_saved


def load_view(folder):
    folder=Path(folder)
    def read(name):return json.loads((folder/name).read_text(encoding='utf-8'))
    status=read('status.json')
    if status.get('schema') not in SUPPORTED_SCHEMAS or status.get('status') not in ('verified_complete','verified_with_manual_residual'):
        raise PlanningError('The selected output is not a finished verified release result')
    cfg=Config.from_dict(read('config.json'));robot=Robot(**read('robot.json')['simulation_parameters']).validate()
    validation=validate_saved(folder)
    if not validation['valid']:
        raise PlanningError('Saved trajectory failed independent revalidation: '+str(validation['errors'][:4]))
    world=World(FieldMap.load(folder/'map.geojson'),robot,cfg.manual_setback_m)
    grid=read('grid.geojson');cells=[];positions={}
    for f in grid['features']:
        p=f['properties'];g=shape(f['geometry'])
        cells.append(SimpleNamespace(polygon=g,id=p['id']))
        if p['traversable']:positions[p['id']]=p['center']
    gd=SimpleNamespace(cells=cells,positions=positions,metadata=grid['metadata'])
    order=SimpleNamespace(**read('ordering.json'))
    areas={f['properties']['kind']:shape(f['geometry']) for f in read('coverage.geojson')['features']}
    return SimpleNamespace(config=cfg,robot=robot,world=world,grid=gd,order=order,areas=areas,
        legs=read('raw_legs.json') if (folder/'raw_legs.json').exists() else [],
        optimization=read('optimization.json'),events=read('events.json'),metrics=read('metrics.json'),validation=validation,folder=folder)
