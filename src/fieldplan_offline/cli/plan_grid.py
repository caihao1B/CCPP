from __future__ import annotations
import argparse
from pathlib import Path

from fieldplan_offline.io.geojson_loader import load_field_geojson
from fieldplan_offline.io.validate_geom import make_valid_polygons
from fieldplan_offline.planners.grid_pipeline import GridPlanConfig, build_plan_from_grid
from fieldplan_offline.export.export_plan_json import save_plan
from fieldplan_offline.export.export_path_csv import save_flat_path_csv

def main():
    ap = argparse.ArgumentParser("Generate plan.json using square-grid + clustering/TSP pipeline")
    ap.add_argument("--geojson", type=Path, required=True)
    ap.add_argument("--cell", type=float, default=0.5)
    ap.add_argument("--out", type=Path, default=Path("plan.json"))
    ap.add_argument("--out_csv", type=Path, default=None)
    ap.add_argument("--cluster_max", type=int, default=120)
    ap.add_argument("--cluster_k", type=int, default=80)
    ap.add_argument("--cluster_min", type=int, default=8)
    ap.add_argument("--stitch_gamma", type=int, default=1000)
    ap.add_argument("--no_patch", action="store_true")
    ap.add_argument("--intra_solver", choices=["ga","christofides"], default="christofides")
    args = ap.parse_args()

    field, obstacles, meta = load_field_geojson(args.geojson)
    obstacles = make_valid_polygons(obstacles)

    cfg = GridPlanConfig(
        cell_size=args.cell,
        cluster_max=args.cluster_max,
        cluster_target=args.cluster_k,
        cluster_min=args.cluster_min,
        stitch_gamma=args.stitch_gamma,
        stitch_patch=not args.no_patch,
        intra_solver=args.intra_solver,
    )

    plan = build_plan_from_grid(field, obstacles, cfg=cfg)
    plan.field_meta.update(meta)

    save_plan(plan, args.out)
    print(f"[ok] wrote {args.out}")

    if args.out_csv:
        save_flat_path_csv(plan, args.out_csv)
        print(f"[ok] wrote {args.out_csv}")

if __name__ == "__main__":
    main()
