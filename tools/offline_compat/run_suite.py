#!/usr/bin/env python3
"""Repeat synthetic and optional repository audits; expected rejections are success."""
import argparse
from importlib.metadata import version
import json
from pathlib import Path
import platform
import subprocess
import sys

from make_examples import create
from validate import read_json, sha


def run(out, name, flags, expected):
    command=[sys.executable,str(Path(__file__).with_name("validate.py")),
             *map(str,flags),"--out",str(out/name)]
    proc=subprocess.run(command,check=False,capture_output=True,text=True)
    if proc.returncode!=expected:
        raise RuntimeError(f"{name}: expected exit {expected}, got {proc.returncode}\n{proc.stdout}\n{proc.stderr}")
    r=read_json(out/name/"report.json")
    record=dict(status=r["status"],exit_code=proc.returncode,command=command,
        input_hashes={k:v["sha256"] for k,v in r.get("inputs",{}).items()},
        before={k:v for k,v in r.get("before",{}).items() if k not in ("segments","seams")},
        after={k:v for k,v in r.get("after",{}).items() if k not in ("segments","seams")},
        delta=r.get("delta"),findings=[{k:v for k,v in f.items() if k!="examples"} for f in r["findings"]])
    print(f"{name}: {r['status']} (expected)")
    return record


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out",required=True,type=Path)
    p.add_argument("--ccpp-root",type=Path)
    p.add_argument("--ros-root",type=Path)
    p.add_argument("--pr1-plan",type=Path)
    args=p.parse_args()
    if (args.ccpp_root is None)!=(args.ros_root is None):
        p.error("Repository audits need both --ccpp-root and --ros-root")
    if args.pr1_plan and not args.ccpp_root:p.error("--pr1-plan requires both repositories")
    examples=create(args.out/"fixtures")
    results={}
    for name in ("safe","short_turn","gap","orientation","stale_length","near_duplicate"):
        flags=["--plan",examples/f"{name}.json","--source-geojson",examples/"field.geojson",
            "--target-geojson",examples/"field.geojson","--map-yaml",examples/"map.yaml",
            "--manifest",examples/f"{name}.manifest.json"]
        results[name]=run(args.out,name,flags,0 if name=="safe" else 1)
        if name=="short_turn":
            results["short_turn_merge_off"]=run(args.out,"short_turn_merge_off",flags+["--merge","off"],0)
    if args.ccpp_root:
        c,r=args.ccpp_root,args.ros_root
        common=["--profile",r/"profiles/executor_e1_static.yaml",
            "--ros-plan-io",r/"src/coverage_manager/coverage_manager/plan_io.py",
            "--source-geojson",c/"field.geojson",
            "--target-geojson",r/"datasets/raw/maps_30/geojson/low_01_5p0m_scatter.geojson",
            "--map-yaml",r/"e1_assets/maps/low_01_5p0m_scatter.yaml"]
        for mode in ("on","off"):
            name=f"historical_merge_{mode}"
            results[name]=run(args.out,name,["--plan",c/"plan_boustrophedon.json",*common,"--merge",mode],1)
        if args.pr1_plan:
            results["pr1_bcd_to_e1"]=run(args.out,"pr1_bcd_to_e1",["--plan",args.pr1_plan,*common],1)
    summary=dict(simulation_tested=False,python=platform.python_version(),
        dependencies={k:version(k) for k in ("shapely","PyYAML","Pillow","numpy")},
        provenance=read_json(Path(__file__).with_name("provenance.json")),results=results)
    args.out.mkdir(parents=True,exist_ok=True)
    (args.out/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")


if __name__=="__main__":main()
