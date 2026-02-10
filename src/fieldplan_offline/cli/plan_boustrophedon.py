from __future__ import annotations
import argparse
from pathlib import Path
import math

from fieldplan_offline.io.geojson_loader import load_field_geojson
from fieldplan_offline.io.validate_geom import make_valid_polygons
from fieldplan_offline.planners.boustrophedon_continuous import compute_boustrophedon_segments
from fieldplan_offline.postprocess.headings import xy_to_poses
from fieldplan_offline.export.plan_schema import Plan, Segment
from fieldplan_offline.export.export_plan_json import save_plan
from fieldplan_offline.export.export_path_csv import save_flat_path_csv


def _straight_connector(a, b, step: float) -> list[tuple[float, float]]:
    ax, ay = a
    bx, by = b
    d = math.hypot(bx-ax, by-ay)
    if d < 1e-9:
        return [a]
    if step <= 0:
        return [a, b]
    n = max(2, int(math.ceil(d / step)) + 1)
    return [(ax + (bx-ax)*i/(n-1), ay + (by-ay)*i/(n-1)) for i in range(n)]


def main():
    ap = argparse.ArgumentParser("Generate plan.json using continuous boustrophedon (swaths)")
    ap.add_argument("--geojson", type=Path, required=True)

    ap.add_argument("--spacing", type=float, default=1.0, help="Swath spacing (m), typically tool width")
    ap.add_argument("--margin", type=float, default=0.0, help="Safety margin (m) to shrink field and inflate obstacles")
    ap.add_argument("--step", type=float, default=1.0, help="Sampling step along each row segment (m)")
    ap.add_argument("--turn_step", type=float, default=1.0, help="Sampling step along turn connectors (m)")
    ap.add_argument("--connect", choices=["straight", "none"], default="straight", help="How to connect consecutive rows")

    ap.add_argument("--out", type=Path, default=Path("plan.json"))
    ap.add_argument("--out_csv", type=Path, default=None)

    args = ap.parse_args()

    field, obstacles, meta = load_field_geojson(args.geojson)
    obstacles = make_valid_polygons(obstacles)

    row_segs = compute_boustrophedon_segments(
        field,
        obstacles,
        swath_spacing=args.spacing,
        margin=args.margin,
        sample_step=args.step,
    )

    segments: list[Segment] = []
    sid = 0
    for i, pts in enumerate(row_segs):
        if len(pts) < 2:
            continue
        poses = xy_to_poses(pts, frame_id="map")
        segments.append(Segment(
            id=f"seg_{sid:04d}",
            kind="row",
            cluster_id=-1,
            frame_id="map",
            poses=poses,
            meta={"row_idx": i, "points": len(pts)},
        ))
        sid += 1

        if args.connect == "straight" and i < len(row_segs) - 1:
            a = pts[-1]
            b = row_segs[i + 1][0] if row_segs[i + 1] else a
            conn = _straight_connector(a, b, step=args.turn_step)
            if len(conn) >= 2:
                poses_t = xy_to_poses(conn, frame_id="map")
                segments.append(Segment(
                    id=f"seg_{sid:04d}",
                    kind="turn",
                    cluster_id=-1,
                    frame_id="map",
                    poses=poses_t,
                    meta={"from_row": i, "to_row": i + 1, "points": len(conn)},
                ))
                sid += 1

    plan = Plan(
        frame_id="map",
        field_meta={"planner": "boustrophedon", "spacing": args.spacing, "margin": args.margin, **meta},
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
