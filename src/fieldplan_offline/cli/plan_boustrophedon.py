from __future__ import annotations
import argparse
from pathlib import Path

from fieldplan_offline.io.geojson_loader import load_field_geojson
from fieldplan_offline.io.validate_geom import make_valid_polygons
from fieldplan_offline.planners.boustrophedon_continuous import compute_boustrophedon_segments
from fieldplan_offline.postprocess.headings import xy_to_poses
from fieldplan_offline.export.plan_schema import Plan, Segment
from fieldplan_offline.export.export_plan_json import save_plan

def main():
    ap = argparse.ArgumentParser("Generate plan.json using continuous boustrophedon (swaths)")
    ap.add_argument("--geojson", type=Path, required=True)
    ap.add_argument("--spacing", type=float, default=1.0)
    ap.add_argument("--margin", type=float, default=0.0)
    ap.add_argument("--step", type=float, default=1.0)
    ap.add_argument("--out", type=Path, default=Path("plan.json"))
    args = ap.parse_args()

    field, obstacles, meta = load_field_geojson(args.geojson)
    obstacles = make_valid_polygons(obstacles)

    seg_xy = compute_boustrophedon_segments(
        field, obstacles,
        swath_spacing=args.spacing,
        margin=args.margin,
        sample_step=args.step,
    )

    segments = []
    for i, pts in enumerate(seg_xy):
        poses = xy_to_poses(pts, frame_id="map")
        segments.append(Segment(
            id=f"row_{i:04d}",
            kind="row",
            cluster_id=-1,
            frame_id="map",
            poses=poses,
            meta={"points": len(pts)}
        ))

    plan = Plan(
        frame_id="map",
        field_meta={"planner": "boustrophedon", "spacing": args.spacing, "margin": args.margin, **meta},
        clusters=[],
        cluster_order=[],
        segments=segments,
        transitions=[]
    )
    save_plan(plan, args.out)
    print(f"[ok] wrote {args.out}")

if __name__ == "__main__":
    main()
