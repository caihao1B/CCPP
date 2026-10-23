#!/usr/bin/env python3
"""Read-only CCPP -> coverage_executor compatibility audit; never launches ROS."""
from __future__ import annotations

import argparse
import ast
from collections import Counter
import copy
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

import yaml

from legacy_merge import compute_segment_length_m as length, merge_short_segments

KINDS = {"intra_cluster", "bridge", "row", "turn", "patch"}
HERE = Path(__file__).resolve().parent


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_json(path):
    # JSON NaN/Infinity are not accepted, even though Python's decoder permits them.
    return json.loads(Path(path).read_text(encoding="utf-8"),
                      parse_constant=lambda s: (_ for _ in ()).throw(ValueError(s)))


def finite(v):
    return isinstance(v, (float, int)) and not isinstance(v, bool) and math.isfinite(v)


def xy(p):
    return float(p["x"]), float(p["y"])


def distance(a, b):
    return math.dist(xy(a), xy(b))


def angle(a, b):
    return abs(math.atan2(math.sin(a-b), math.cos(a-b)))


def yaw(p):
    if all(k in p for k in ("qx", "qy", "qz", "qw")):
        x, y, z, w = (p[k] for k in ("qx", "qy", "qz", "qw"))
        return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    return p.get("yaw", 0.0)


class Findings:
    def __init__(self):
        self.data = {}

    def add(self, code, origin, detail, block=True):
        key = (code, origin, block)
        item = self.data.setdefault(key, dict(code=code, origin=origin,
            blocks_import=block, count=0, examples=[]))
        item["count"] += 1
        if len(item["examples"]) < 10:
            item["examples"].append(detail)

    def items(self):
        return list(self.data.values())

    def blocked(self):
        return any(i["blocks_import"] for i in self.data.values())


def schema(plan, findings):
    """Reject ambiguous data before the permissive ROS reader can coerce it."""
    def bad(code, detail):
        findings.add(code, "input_schema", detail)

    if not isinstance(plan, dict):
        bad("PLAN_NOT_OBJECT", "root"); return False
    frame = plan.get("frame_id")
    if not isinstance(frame, str) or not frame.strip():
        bad("FRAME_REQUIRED", "plan.frame_id")
    for name, kind in (("field_meta", dict), ("clusters", list),
                       ("cluster_order", list), ("transitions", list)):
        if name not in plan:
            findings.add("CCPP_OPTIONAL_FIELD_ABSENT", "input_schema", name, False)
        elif not isinstance(plan[name], kind):
            bad("METADATA_STRUCTURE", name)
    if isinstance(plan.get("transitions"), list) and any(
            not isinstance(t, dict) for t in plan["transitions"]):
        bad("METADATA_STRUCTURE", "transitions entries must be objects")
    if isinstance(plan.get("clusters"), list) and any(
            not isinstance(c, list) or any(type(n) is not int for n in c)
            for c in plan["clusters"]):
        bad("METADATA_STRUCTURE", "clusters must contain integer lists")
    if isinstance(plan.get("cluster_order"), list) and any(
            type(n) is not int for n in plan["cluster_order"]):
        bad("METADATA_STRUCTURE", "cluster_order must contain integers")
    segs = plan.get("segments")
    if not isinstance(segs, list) or not segs:
        bad("EMPTY_OR_INVALID_SEGMENTS", "segments"); return False
    ids = set()
    for i, seg in enumerate(segs):
        if not isinstance(seg, dict):
            bad("SEGMENT_NOT_OBJECT", i); continue
        sid = seg.get("id")
        if not isinstance(sid, str) or not sid or sid in ids:
            bad("SEGMENT_ID", i)
        else:
            ids.add(sid)
        if seg.get("kind") not in KINDS:
            bad("SEGMENT_KIND", i)
        if "cluster_id" in seg and type(seg["cluster_id"]) is not int:
            bad("METADATA_STRUCTURE", f"segments[{i}].cluster_id")
        if seg.get("frame_id", frame) != frame:
            bad("FRAME_MISMATCH", f"segments[{i}].frame_id")
        meta = seg.get("meta", {})
        if not isinstance(meta, dict):
            bad("METADATA_STRUCTURE", f"segments[{i}].meta")
        else:
            if "length_m" in meta and (not finite(meta["length_m"]) or meta["length_m"] < 0):
                bad("INVALID_DECLARED_LENGTH", i)
            if "merged_from" in meta and not isinstance(meta["merged_from"], list):
                bad("METADATA_STRUCTURE", f"segments[{i}].meta.merged_from")
        poses = seg.get("poses")
        if not isinstance(poses, list) or not poses:
            bad("EMPTY_OR_INVALID_POSES", i); continue
        for j, p in enumerate(poses):
            where = f"segments[{i}].poses[{j}]"
            if not isinstance(p, dict):
                bad("POSE_NOT_OBJECT", where); continue
            if any(not finite(p.get(k)) for k in ("x", "y")):
                bad("INVALID_POSITION", where)
            if p.get("frame_id", seg.get("frame_id", frame)) != frame:
                bad("FRAME_MISMATCH", where)
            if "z" in p and (not finite(p["z"]) or abs(p["z"]) > 1e-6):
                bad("NONPLANAR_POSITION", where)
            keys = [k in p for k in ("qx", "qy", "qz", "qw")]
            if any(keys) and not all(keys):
                bad("PARTIAL_QUATERNION", where); continue
            if all(keys):
                q = [p[k] for k in ("qx", "qy", "qz", "qw")]
                if not all(finite(n) for n in q):
                    bad("INVALID_QUATERNION", where); continue
                if abs(math.hypot(*q)-1) > 1e-5:
                    bad("NONUNIT_QUATERNION", where)
                if abs(q[0]) > 1e-6 or abs(q[1]) > 1e-6:
                    bad("NONPLANAR_QUATERNION", where)
            elif "yaw" not in p:
                bad("ORIENTATION_REQUIRED", where)
            else:
                findings.add("YAW_FALLBACK", "input_schema", where, False)
            if "yaw" in p:
                if not finite(p["yaw"]):
                    bad("INVALID_YAW", where)
                elif all(keys) and angle(yaw(p), p["yaw"]) > 1e-5:
                    bad("YAW_QUATERNION_CONFLICT", where)
    return not any(i["blocks_import"] for i in findings.items())


def metrics(segs, options):
    rows, seams = [], []
    for i, s in enumerate(segs):
        pts = s["poses"]
        ds = [distance(a, b) for a, b in zip(pts, pts[1:])]
        rows.append(dict(index=i, id=s["id"], kind=s["kind"], poses=len(pts),
            length_m=length(pts), declared_length_m=s.get("meta", {}).get("length_m"),
            max_edge_m=max(ds, default=0),
            duplicate_edges=sum(d == 0 for d in ds),
            near_duplicate_edges=sum(0 < d <= options["duplicate_tolerance_m"] for d in ds)))
        if i:
            seams.append(dict(from_id=segs[i-1]["id"], to_id=s["id"],
                gap_m=distance(segs[i-1]["poses"][-1], pts[0]),
                yaw_change_rad=angle(yaw(segs[i-1]["poses"][-1]), yaw(pts[0]))))
    points = [xy(p) for s in segs for p in s["poses"]]
    return dict(segment_count=len(segs), kinds=dict(Counter(s["kind"] for s in segs)),
        pose_count=sum(len(s["poses"]) for s in segs),
        path_length_m=sum(r["length_m"] for r in rows),
        seam_length_m=sum(s["gap_m"] for s in seams),
        bounds=[min(p[0] for p in points), min(p[1] for p in points),
                max(p[0] for p in points), max(p[1] for p in points)],
        discontinuous_seams=sum(s["gap_m"] > options["join_tolerance_m"] for s in seams),
        duplicate_edges=sum(r["duplicate_edges"] for r in rows),
        near_duplicate_edges=sum(r["near_duplicate_edges"] for r in rows),
        segments=rows, seams=seams)


def audit_geometry_metrics(m, findings, origin, options, original=None):
    old = {s["id"]: s for s in original["segments"]} if original else {}
    for s in m["segments"]:
        detail = dict(id=s["id"], length_m=s["length_m"])
        if s["length_m"] <= options["duplicate_tolerance_m"]:
            findings.add("DEGENERATE_SEGMENT", origin, detail)
        if s["length_m"] > options["max_segment_m"]:
            if not original or old[s["id"]]["length_m"] <= options["max_segment_m"]:
                findings.add("SEGMENT_TOO_LONG", origin, detail)
        if s["declared_length_m"] is not None and abs(s["declared_length_m"]-s["length_m"]) > 1e-6:
            # Do not relabel an untouched original metadata defect as preprocessing damage.
            inherited = original and s["declared_length_m"] == old[s["id"]]["declared_length_m"] and s["length_m"] == old[s["id"]]["length_m"]
            if not inherited:
                findings.add("LENGTH_METADATA_MISMATCH", origin, dict(**detail,
                    declared_length_m=s["declared_length_m"]))
        if not original:
            if s["max_edge_m"] > options["max_edge_m"]:
                findings.add("OVERSIZED_INPUT_EDGE", origin,
                    dict(id=s["id"], max_edge_m=s["max_edge_m"]))
            if s["duplicate_edges"] or s["near_duplicate_edges"]:
                findings.add("DUPLICATE_INPUT_POINTS", origin,
                    dict(id=s["id"], exact=s["duplicate_edges"], near=s["near_duplicate_edges"]), False)
    if not original:
        for s in m["seams"]:
            if s["gap_m"] > options["join_tolerance_m"]:
                findings.add("INPUT_SEAM_GAP", origin, s)


DEFAULTS = dict(merge_enabled=True, length_threshold_m=2.0, min_poses=3,
                join_tolerance_m=1e-6, duplicate_tolerance_m=1e-6,
                max_edge_m=2.0, max_segment_m=100.0, clearance_m=0.0)


def analyze(plan, options=None):
    options = {**DEFAULTS, **(options or {})}
    findings = Findings()
    result = dict(report_version=1, scope="offline_coverage_executor_preprocessing_only",
                  simulation_tested=False, options=options)
    if not schema(plan, findings):
        result.update(status="INVALID_INPUT", findings=findings.items())
        return result, None, findings
    original = copy.deepcopy(plan["segments"])
    before = metrics(original, options)
    audit_geometry_metrics(before, findings, "original_plan", options)
    # Trace decisions independently of user-controlled merged_from metadata.
    groups, joins = [], []
    endpoint = None
    for i, seg in enumerate(original):
        decision_len = seg.get("meta", {}).get("length_m", length(seg["poses"]))
        short = decision_len < options["length_threshold_m"] or len(seg["poses"]) < options["min_poses"]
        if options["merge_enabled"] and short and groups:
            group = groups[-1]
            # Keep the surviving endpoint even when an equal-XY singleton disappears.
            a, b = endpoint, seg["poses"][0]
            join = dict(from_id=original[group[0]]["id"], incoming_id=seg["id"],
                incoming_kind=seg["kind"], retained_kind=original[group[0]]["kind"],
                gap_m=distance(a, b), dropped_first_pose=(xy(a) == xy(b)),
                yaw_change_rad=angle(yaw(a), yaw(b)), decision_length_m=decision_len)
            joins.append(join)
            if join["incoming_kind"] != join["retained_kind"]:
                findings.add("KIND_SEMANTICS_LOST", "executor_merge", join)
            retained = original[group[0]]
            if seg.get("cluster_id") != retained.get("cluster_id"):
                findings.add("CLUSTER_ID_LOST", "executor_merge", dict(
                    incoming_id=seg["id"], incoming_cluster=seg.get("cluster_id"),
                    retained_id=retained["id"], retained_cluster=retained.get("cluster_id")))
            if join["gap_m"] > options["join_tolerance_m"]:
                findings.add("SEAM_PROMOTED_TO_INTERNAL_EDGE", "executor_merge", join)
            if join["dropped_first_pose"] and join["yaw_change_rad"] > 1e-5:
                findings.add("JUNCTION_ORIENTATION_DROPPED", "executor_merge", join)
            if 0 < join["gap_m"] <= options["duplicate_tolerance_m"]:
                findings.add("NEAR_DUPLICATE_JUNCTION_RETAINED", "executor_merge", join, False)
            group.append(i)
            if not join["dropped_first_pose"] or len(seg["poses"]) > 1:
                endpoint = seg["poses"][-1]
        else:
            groups.append([i])
            endpoint = seg["poses"][-1]
    after_segs = (merge_short_segments(copy.deepcopy(original), options["length_threshold_m"],
                  options["min_poses"]) if options["merge_enabled"] else copy.deepcopy(original))
    after = metrics(after_segs, options)
    audit_geometry_metrics(after, findings, "executor_merge", options, before)
    mapping = [dict(output_index=i, output_id=after_segs[i]["id"],
                    output_kind=after_segs[i]["kind"],
                    sources=[dict(index=n, id=original[n]["id"], kind=original[n]["kind"],
                                  cluster_id=original[n].get("cluster_id"),
                                  frame_id=original[n].get("frame_id",plan["frame_id"]),
                                  meta=original[n].get("meta", {})) for n in g]) for i, g in enumerate(groups)]
    # Source ownership survives in this report, never injected into the exported Plan.
    result.update(before=before, after=after, merge_operations=joins, segment_mapping=mapping,
        disappeared_event_ids=[original[n]["id"] for g in groups for n in g[1:]],
        delta=dict(segments=after["segment_count"]-before["segment_count"],
                   poses=after["pose_count"]-before["pose_count"],
                   internal_length_m=after["path_length_m"]-before["path_length_m"],
                   total_length_including_seams_m=(after["path_length_m"]+after["seam_length_m"]
                       - before["path_length_m"]-before["seam_length_m"])),
        metadata=dict(plan_keys=sorted(plan), field_meta=plan.get("field_meta", {}),
                      transitions_count=len(plan.get("transitions", [])),
                      note="coverage_executor does not remap top-level transitions/clusters; absorbed segment metadata is not propagated"))
    if plan.get("transitions") and joins:
        findings.add("TRANSITIONS_REQUIRE_REVIEW", "executor_merge", "Top-level transition references are not rewritten")
    result.update(status="BLOCKED" if findings.blocked() else "MAP_EVIDENCE_REQUIRED", findings=findings.items())
    return result, after_segs, findings


def profile_options(path):
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    p = data["coverage_executor"]["ros__parameters"]
    enabled, threshold, minimum = (p[k] for k in
        ("merge_short_segments", "merge_length_threshold_m", "merge_min_poses"))
    if type(enabled) is not bool or not finite(threshold) or threshold < 0 or type(minimum) is not int or minimum < 1:
        raise ValueError("Invalid merge profile types/ranges")
    if p.get("use_smoother", False):
        raise ValueError("Smoother is outside this offline model; use_smoother must be false")
    return dict(merge_enabled=enabled, length_threshold_m=threshold, min_poses=minimum)


def verify_upstream(path):
    expected = read_json(HERE/"provenance.json")["plan_io_sha256"]
    if sha(path) != expected:
        raise ValueError("plan_io.py hash differs from audited source; re-audit before importing")
    names = {"_dist", "compute_segment_length_m", "merge_short_segments"}
    def pure(source):
        return [ast.dump(n, include_attributes=False) for n in ast.parse(source).body
                if isinstance(n, ast.FunctionDef) and n.name in names]
    if pure(Path(path).read_text()) != pure((HERE/"legacy_merge.py").read_text()):
        raise ValueError("Legacy excerpt no longer matches upstream AST")


def write_reports(result, out):
    out.mkdir(parents=True, exist_ok=True)
    (out/"report.json").write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    for stage in ("before", "after"):
        rows = result.get(stage, {}).get("segments", [])
        if rows:
            with (out/f"{stage}_segments.csv").open("w", newline="", encoding="utf-8") as f:
                w = csv.DictWriter(f, fieldnames=list(rows[0]), lineterminator="\n"); w.writeheader(); w.writerows(rows)
    lines = ["# Offline CCPP → ROS2 compatibility audit", "", f"Status: **{result['status']}**",
             "", "ROS 2 / Gazebo / Nav2 execution: **NOT TESTED**.", ""]
    if "before" in result:
        lines += ["| Metric | Before | After |", "|---|---:|---:|"]
        for k in ("segment_count", "pose_count", "path_length_m", "seam_length_m", "duplicate_edges", "discontinuous_seams"):
            lines.append(f"| {k} | {result['before'][k]} | {result['after'][k]} |")
    lines += ["", "| Finding | Origin | Count | Blocks import |", "|---|---|---:|---|"]
    for f in result.get("findings", []):
        lines.append(f"| {f['code']} | {f['origin']} | {f['count']} | {f['blocks_import']} |")
    lines += ["", "Full IDs, kinds, lengths, seams and source mappings are in report.json and the CSV files.",
              "No executable plan is written and no coordinates are transformed."]
    (out/"report.md").write_text("\n".join(lines)+"\n", encoding="utf-8")


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--plan", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--profile", type=Path)
    p.add_argument("--ros-plan-io", type=Path)
    p.add_argument("--merge", choices=("on", "off"), help="Audit-only override; never edits YAML")
    p.add_argument("--source-geojson", type=Path)
    p.add_argument("--target-geojson", type=Path)
    p.add_argument("--map-yaml", type=Path)
    p.add_argument("--manifest", type=Path)
    for k in ("max_edge_m", "max_segment_m", "join_tolerance_m", "duplicate_tolerance_m", "clearance_m"):
        p.add_argument("--"+k.replace("_", "-"), type=float, default=DEFAULTS[k])
    args = p.parse_args(argv)
    # Never allow report files to overwrite any caller-specified input.
    inputs = [v.resolve() for v in vars(args).values() if isinstance(v, Path) and v != args.out]
    if any((args.out/name).resolve() in inputs for name in ("report.json", "report.md", "before_segments.csv", "after_segments.csv")):
        p.error("Output would overwrite an input")
    try:
        opts = {k:getattr(args,k) for k in DEFAULTS if hasattr(args,k)}
        if any(not finite(v) or v < 0 for v in opts.values()):
            raise ValueError("Audit limits must be finite and non-negative")
        if args.profile:
            opts.update(profile_options(args.profile))
        if args.merge:
            opts["merge_enabled"] = args.merge == "on"
        if args.ros_plan_io:
            verify_upstream(args.ros_plan_io)
        plan = read_json(args.plan)
        result, after, findings = analyze(plan, opts)
        result["inputs"] = {k:dict(path=str(v), sha256=sha(v)) for k,v in vars(args).items()
                            if isinstance(v,Path) and k != "out"}
        result["provenance"] = read_json(HERE/"provenance.json")
        result["upstream_verified"] = bool(args.ros_plan_io)
        if after is not None:
            from map_checks import check_maps
            result["maps"] = check_maps(plan, after, args, result["options"], findings)
            result["status"] = "BLOCKED" if findings.blocked() else "OFFLINE_PASS"
            result["findings"] = findings.items()
        write_reports(result, args.out)
        print(json.dumps(dict(status=result["status"], report=str(args.out/"report.json"),
                              simulation_tested=False)))
        return 2 if result["status"] == "INVALID_INPUT" else (1 if result["status"] == "BLOCKED" else 0)
    except (ValueError, TypeError, KeyError, OSError, yaml.YAMLError) as e:
        result = dict(status="INVALID_INPUT", simulation_tested=False, error=str(e), findings=[])
        write_reports(result, args.out)
        print(f"INVALID_INPUT: {e}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
