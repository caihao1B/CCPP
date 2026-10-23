"""Create deterministic, clearly synthetic fixtures; never modify input datasets."""
import argparse
import copy
import json
import math
from pathlib import Path

from PIL import Image

from validate import sha


def pose(x,y,yaw=0):
    return dict(x=x,y=y,yaw=yaw,qx=0.,qy=0.,qz=math.sin(yaw/2),qw=math.cos(yaw/2),frame_id="map")


def segment(sid,kind,points,meta=None):
    return dict(id=sid,kind=kind,frame_id="map",cluster_id=-1,
                poses=[pose(*p) for p in points], meta={} if meta is None else meta)


def plan(segments):
    return dict(frame_id="map",field_meta={"synthetic":True},clusters=[],cluster_order=[],
                transitions=[],segments=segments)


def cases():
    row=segment("row_a","row",[(1,1),(2,1),(3,1)])
    turn=segment("turn_b","turn",[(3,1),(3,1.25),(3,1.5)])
    next_row=segment("row_c","row",[(3,1.5),(2,1.5),(1,1.5)])
    continuous=plan([row,turn,next_row])
    gap=copy.deepcopy(continuous)
    gap["segments"][1]["poses"]=[pose(4,1),pose(4,1.25),pose(4,1.5)]
    orientation=copy.deepcopy(continuous)
    orientation["segments"][1]["poses"][0]=pose(3,1,math.pi/2)
    stale=copy.deepcopy(continuous)
    stale["segments"][1]["meta"]["length_m"]=20.
    near=copy.deepcopy(continuous)
    near["segments"][1]["poses"][0]["x"]+=1e-9
    return dict(safe=plan([row]),short_turn=continuous,gap=gap,
                orientation=orientation,stale_length=stale,near_duplicate=near)


def geometry(obstacles=None):
    def feature(kind,ring):
        return dict(type="Feature",properties={"kind":kind},
                    geometry=dict(type="Polygon",coordinates=[ring]))
    return dict(type="FeatureCollection",features=[feature("field",[[0,0],[10,0],[10,10],[0,10],[0,0]])]
                + [feature("obstacle",o) for o in (obstacles or [])])


def dump(path,value):
    Path(path).write_text(json.dumps(value,indent=2)+"\n",encoding="utf-8")


def create(out):
    out=Path(out); out.mkdir(parents=True,exist_ok=True)
    for name,value in cases().items():
        dump(out/f"{name}.json",value)
    dump(out/"field.geojson",geometry())
    # 1 m occupied padding around a 10 x 10 m synthetic field, 0.1 m cells.
    im=Image.new("L",(120,120),0)
    im.paste(254,(10,10,110,110)); im.save(out/"map.pgm")
    (out/"map.yaml").write_text("image: map.pgm\nmode: trinary\nresolution: 0.1\norigin: [-1.0, -1.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    for name in cases():
        paths=dict(plan=out/f"{name}.json",source_geojson=out/"field.geojson",
                   target_geojson=out/"field.geojson",map_yaml=out/"map.yaml",map_image=out/"map.pgm")
        dump(out/f"{name}.manifest.json",dict(version=1,units="m",axes="x_y_right_handed",
            frame_id="map",source_to_target_se2=[0,0,0],
            coordinate_review="SYNTHETIC ONLY: Cartesian metres, source and target generated in one coordinate system; no real Gazebo evidence.",
            sha256={k:sha(v) for k,v in paths.items()}))
    return out


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("--out",type=Path,required=True)
    create(p.parse_args().out)
