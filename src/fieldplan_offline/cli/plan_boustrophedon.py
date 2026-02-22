from __future__ import annotations

import argparse
import math
from pathlib import Path

from fieldplan_offline.export.export_path_csv import save_flat_path_csv
from fieldplan_offline.export.export_plan_json import save_plan
from fieldplan_offline.export.plan_schema import Plan, Segment
from fieldplan_offline.io.geojson_loader import load_field_geojson
from fieldplan_offline.io.validate_geom import make_valid_polygons
from fieldplan_offline.planners.boustrophedon_bcd import plan_boustrophedon_bcd
from fieldplan_offline.planners.boustrophedon_continuous import compute_boustrophedon_segments
from fieldplan_offline.postprocess.headings import xy_to_poses


def _straight_connector(a, b, step: float) -> list[tuple[float, float]]:
    ax, ay = a
    bx, by = b
    d = math.hypot(bx - ax, by - ay)
    if d < 1e-9:
        return [a]
    if step <= 0:
        return [a, b]
    n = max(2, int(math.ceil(d / step)) + 1)
    return [(ax + (bx - ax) * i / (n - 1), ay + (by - ay) * i / (n - 1)) for i in range(n)]


def main():
    ap = argparse.ArgumentParser("Generate plan.json using boustrophedon coverage planners")
    ap.add_argument("--geojson", type=Path, required=True)

    # common
    ap.add_argument("--spacing", type=float, default=1.0, help="Swath spacing (m), typically tool width")
    ap.add_argument("--margin", type=float, default=0.0, help="Safety margin (m) to shrink field and inflate obstacles")
    ap.add_argument("--step", type=float, default=1.0, help="Sampling step along rows (m)")
    ap.add_argument("--out", type=Path, default=Path("plan.json"))
    ap.add_argument("--out_csv", type=Path, default=None)

    # choose method
    ap.add_argument(
        "--method",
        choices=["continuous", "bcd"],
        default="continuous",
        help="continuous: simple serpentine over whole free space; bcd: boustrophedon cellular decomposition",
    )

    # continuous connectors
    ap.add_argument("--connect", choices=["straight", "none"], default="straight")
    ap.add_argument("--turn_step", type=float, default=1.0)

    # BCD options
    ap.add_argument("--order", choices=["dfs", "tsp"], default="dfs", help="Cell traversal order for BCD")
    ap.add_argument("--sweep_angle", type=float, default=None, help="Override sweep angle (deg)")
    ap.add_argument("--connect_cell", type=float, default=None, help="Connector grid cell size (m)")
    ap.add_argument("--connect_step", type=float, default=None, help="Sampling step for internal connector densify (m)")
    ap.add_argument("--crit_tol", type=float, default=None, help="Critical x quantization tol (m)")
    ap.add_argument("--slab_min_width", type=float, default=None, help="Minimum slab width (m)")
    ap.add_argument("--min_cell_area", type=float, default=1e-3, help="Minimum cell area (m^2)")
    ap.add_argument("--adj_tol", type=float, default=None, help="Adjacency tolerance (m)")

    args = ap.parse_args()

    field, obstacles, meta = load_field_geojson(args.geojson)
    obstacles = make_valid_polygons(obstacles)

    segments: list[Segment] = []
    sid = 0

    if args.method == "continuous":
        row_segs = compute_boustrophedon_segments(
            field,
            obstacles,
            swath_spacing=args.spacing,
            margin=args.margin,
            sample_step=args.step,
        )

        for i, pts in enumerate(row_segs):
            if len(pts) < 2:
                continue

            segments.append(
                Segment(
                    id=f"seg_{sid:04d}",
                    kind="row",
                    cluster_id=-1,
                    frame_id="map",
                    poses=xy_to_poses(pts, frame_id="map"),
                    meta={"row_idx": i, "points": len(pts), "method": "continuous"},
                )
            )
            sid += 1

            if args.connect == "straight" and i < len(row_segs) - 1:
                a = pts[-1]
                b = row_segs[i + 1][0] if row_segs[i + 1] else a
                conn = _straight_connector(a, b, step=args.turn_step)
                if len(conn) >= 2:
                    segments.append(
                        Segment(
                            id=f"seg_{sid:04d}",
                            kind="turn",
                            cluster_id=-1,
                            frame_id="map",
                            poses=xy_to_poses(conn, frame_id="map"),
                            meta={"from_row": i, "to_row": i + 1, "points": len(conn), "method": "continuous"},
                        )
                    )
                    sid += 1

        plan_meta = {"planner": "boustrophedon_continuous", "spacing": args.spacing, "margin": args.margin, **meta}

    else:
        seg_specs, extra = plan_boustrophedon_bcd(
            field,
            obstacles,
            swath_spacing=args.spacing,
            margin=args.margin,
            sweep_angle_deg=args.sweep_angle,
            sample_step=args.step,
            connector_cell_size=args.connect_cell,
            connector_step=args.connect_step,
            cell_order_mode=args.order,
            crit_tol=args.crit_tol,
            min_slab_width=args.slab_min_width,
            min_cell_area=args.min_cell_area,
            adjacency_tol=args.adj_tol,
        )

        for i, s in enumerate(seg_specs):
            pts = s["points"]
            if len(pts) < 2:
                continue
            segments.append(
                Segment(
                    id=f"seg_{sid:04d}",
                    kind=s["kind"],  # "row" or "turn"
                    cluster_id=-1,
                    frame_id="map",
                    poses=xy_to_poses(pts, frame_id="map"),
                    meta={"seg_idx": i, **s.get("meta", {}), "method": "bcd"},
                )
            )
            sid += 1

        plan_meta = {**meta, **extra}

    plan = Plan(
        frame_id="map",
        field_meta=plan_meta,
        clusters=[],
        cluster_order=[],
        segments=segments,
        transitions=[],
    )

    save_plan(plan, args.out)
    print(f"[ok] wrote {args.out}")

    if args.out_csv:
        save_flat_path_csv(plan, args.out_csv)
        print(f"[ok] wrote {args.out_csv}")


if __name__ == "__main__":
    main()
