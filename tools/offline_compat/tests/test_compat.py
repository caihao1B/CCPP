import contextlib
import copy
import io
import json
import math
from pathlib import Path
import random
import sys
import tempfile
from types import SimpleNamespace
import unittest

from PIL import Image
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from validate import analyze, Findings, main, read_json, sha, verify_upstream
from legacy_merge import merge_short_segments
from make_examples import cases, create, dump, geometry, plan, pose, segment
from map_checks import check_maps, field_geometry, geometry_failures, raster, raster_check


def codes(report):
    return {f["code"] for f in report["findings"]}


class MergeTests(unittest.TestCase):
    def test_short_turn_counts_and_length_bug(self):
        r, output, _ = analyze(cases()["short_turn"])
        self.assertEqual(r["before"]["segment_count"],3)
        self.assertEqual(r["after"]["segment_count"],2)
        self.assertEqual(output[0]["id"],"row_a")
        self.assertEqual(output[0]["kind"],"row")
        self.assertAlmostEqual(r["after"]["segments"][0]["length_m"],2.5)
        self.assertAlmostEqual(output[0]["meta"]["length_m"],3.)
        self.assertIn("LENGTH_METADATA_MISMATCH",codes(r))
        self.assertEqual(r["disappeared_event_ids"],["turn_b"])
        self.assertEqual([s["id"] for s in r["segment_mapping"][0]["sources"]],["row_a","turn_b"])

    def test_source_is_not_mutated(self):
        p=cases()["short_turn"]; expected=copy.deepcopy(p)
        analyze(p); self.assertEqual(p,expected)

    def test_disabled_merge(self):
        p=cases()["short_turn"]
        r,out,_=analyze(p,{"merge_enabled":False})
        self.assertEqual(out,p["segments"])
        self.assertEqual(r["merge_operations"],[])

    def test_first_short_kept(self):
        p=plan([segment("first","turn",[(0,0)])])
        r,out,_=analyze(p)
        self.assertEqual(len(out),1)
        self.assertIn("DEGENERATE_SEGMENT",codes(r))

    def test_strict_threshold_and_min_poses(self):
        p=plan([segment("a","row",[(0,0),(1,0),(2,0)]),
                segment("b","row",[(2,0),(3,0),(4,0)])])
        self.assertEqual(analyze(p)[0]["after"]["segment_count"],2)

    def test_long_two_point_segment_merges(self):
        p=plan([segment("a","row",[(0,0),(1,0),(2,0)]),
                segment("b","turn",[(2,0),(102,0)])])
        r,out,_=analyze(p)
        self.assertEqual(len(out),1)
        self.assertIn("OVERSIZED_INPUT_EDGE",codes(r))
        self.assertIn("SEGMENT_TOO_LONG",codes(r))

    def test_stale_length_controls_decision(self):
        r,out,_=analyze(cases()["stale_length"])
        self.assertEqual(len(out),3)
        self.assertIn("LENGTH_METADATA_MISMATCH",codes(r))
        self.assertTrue(all(f["origin"]=="original_plan" for f in r["findings"]
                            if f["code"]=="LENGTH_METADATA_MISMATCH"))

    def test_valid_length_metadata_avoids_double_count(self):
        p=cases()["short_turn"]
        p["segments"][0]["meta"]["length_m"]=2.
        r,out,_=analyze(p)
        self.assertEqual(out[0]["meta"]["length_m"],2.5)
        self.assertNotIn("LENGTH_METADATA_MISMATCH",codes(r))

    def test_gap_is_original_then_promoted(self):
        r,out,_=analyze(cases()["gap"])
        self.assertIn("INPUT_SEAM_GAP",codes(r))
        self.assertIn("SEAM_PROMOTED_TO_INTERNAL_EDGE",codes(r))
        self.assertAlmostEqual(r["delta"]["total_length_including_seams_m"],0.)
        self.assertAlmostEqual(r["delta"]["internal_length_m"],1.)

    def test_equal_xy_drops_different_orientation(self):
        r,out,_=analyze(cases()["orientation"])
        self.assertIn("JUNCTION_ORIENTATION_DROPPED",codes(r))
        self.assertEqual(out[0]["poses"][2]["yaw"],0.)

    def test_near_equal_not_deduplicated(self):
        r,_,_=analyze(cases()["near_duplicate"])
        self.assertIn("NEAR_DUPLICATE_JUNCTION_RETAINED",codes(r))
        self.assertEqual(r["after"]["near_duplicate_edges"],1)

    def test_existing_duplicates_attributed_to_input(self):
        p=cases()["safe"]
        p["segments"][0]["poses"].insert(1,pose(1,1))
        r,_,_=analyze(p)
        f=next(f for f in r["findings"] if f["code"]=="DUPLICATE_INPUT_POINTS")
        self.assertEqual(f["origin"],"original_plan")

    def test_singleton_trace_keeps_actual_surviving_orientation(self):
        p=plan([segment("a","row",[(0,0),(1,0),(2,0)]),
                segment("b","turn",[(2,0,1)]),segment("c","turn",[(2,0),(2,.2)])])
        r,_,_=analyze(p)
        self.assertEqual(r["merge_operations"][1]["yaw_change_rad"],0.)

    def test_negative_quaternion_equivalent(self):
        p=cases()["orientation"]
        p["segments"][1]["poses"][0]=pose(3,1)
        p["segments"][1]["poses"][0]["qw"]=-1.
        self.assertNotIn("JUNCTION_ORIENTATION_DROPPED",codes(analyze(p)[0]))

    def test_transition_references_require_review(self):
        p=cases()["short_turn"]; p["transitions"]=[{"to":"turn_b"}]
        self.assertIn("TRANSITIONS_REQUIRE_REVIEW",codes(analyze(p)[0]))

    def test_user_merged_from_does_not_forge_mapping(self):
        p=cases()["short_turn"]; p["segments"][0]["meta"]["merged_from"]=["forged"]
        self.assertEqual(analyze(p)[0]["disappeared_event_ids"],["turn_b"])
        p["segments"][1]["cluster_id"]=7
        r,_,_=analyze(p)
        self.assertIn("CLUSTER_ID_LOST",codes(r))
        self.assertEqual(r["segment_mapping"][0]["sources"][1]["cluster_id"],7)

    def test_randomized_trace_matches_legacy_and_conserves_geometric_route(self):
        rng=random.Random(42)
        for _ in range(60):
            ss=[]; x=0.
            for i in range(30):
                x+=rng.choice([0.,0.,.01])
                pts=[(x,0)]
                for _ in range(rng.randint(0,4)):
                    x+=rng.choice([0.,.1,1.]);pts.append((x,0))
                ss.append(segment(str(i),rng.choice(["row","turn"]),pts))
            r,out,_=analyze(plan(ss))
            expected=merge_short_segments(copy.deepcopy(ss),2.,3)
            self.assertEqual(out,expected)
            self.assertEqual(len(r["segment_mapping"]),len(out))
            self.assertEqual(sum(len(g["sources"]) for g in r["segment_mapping"]),len(ss))
            self.assertAlmostEqual(r["delta"]["total_length_including_seams_m"],0.,places=8)


class SchemaTests(unittest.TestCase):
    def test_invalid_schema_table(self):
        changes=[
            (lambda p:p.update(segments=[]),"EMPTY_OR_INVALID_SEGMENTS"),
            (lambda p:p["segments"][0].update(poses=[]),"EMPTY_OR_INVALID_POSES"),
            (lambda p:p["segments"][0].update(meta=None),"METADATA_STRUCTURE"),
            (lambda p:p["segments"][0].update(kind="unknown"),"SEGMENT_KIND"),
            (lambda p:p["segments"][0]["meta"].update(length_m=-1),"INVALID_DECLARED_LENGTH"),
            (lambda p:p["segments"][0]["meta"].update(length_m=float('nan')),"INVALID_DECLARED_LENGTH"),
            (lambda p:p["segments"][0]["poses"][0].update(x="1"),"INVALID_POSITION"),
            (lambda p:p["segments"][0]["poses"][0].update(x=True),"INVALID_POSITION"),
            (lambda p:p["segments"][0]["poses"][0].update(x=float('inf')),"INVALID_POSITION"),
            (lambda p:p["segments"][0]["poses"][0].pop("qw"),"PARTIAL_QUATERNION"),
            (lambda p:p["segments"][0]["poses"][0].update(qw=0),"NONUNIT_QUATERNION"),
            (lambda p:p["segments"][0]["poses"][0].update(qx=1,qw=0),"NONPLANAR_QUATERNION"),
            (lambda p:p["segments"][0]["poses"][0].update(yaw=1),"YAW_QUATERNION_CONFLICT"),
            (lambda p:p["segments"][0]["poses"][0].update(z=1),"NONPLANAR_POSITION"),
            (lambda p:p["segments"][0]["poses"][0].update(frame_id="odom"),"FRAME_MISMATCH"),
            (lambda p:p["segments"][0].update(frame_id="odom"),"FRAME_MISMATCH"),
            (lambda p:p.update(frame_id=""),"FRAME_REQUIRED"),
            (lambda p:p["segments"].append(copy.deepcopy(p["segments"][0])),"SEGMENT_ID"),
            (lambda p:p.update(transitions=[0]),"METADATA_STRUCTURE"),
            (lambda p:p.update(clusters=[1]),"METADATA_STRUCTURE")]
        for change,expected in changes:
            with self.subTest(expected=expected):
                p=cases()["safe"];change(p)
                r,out,_=analyze(p)
                self.assertEqual(r["status"],"INVALID_INPUT");self.assertIsNone(out)
                self.assertIn(expected,codes(r))

    def test_yaw_only_is_supported_but_visible(self):
        p=cases()["safe"]
        for pt in p["segments"][0]["poses"]:
            for k in ("qx","qy","qz","qw"):pt.pop(k)
        r,out,_=analyze(p)
        self.assertIsNotNone(out);self.assertIn("YAW_FALLBACK",codes(r))


class MapTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=create(self.tmp.name)
        self.args=SimpleNamespace(plan=self.root/"safe.json",source_geojson=self.root/"field.geojson",
             target_geojson=self.root/"field.geojson",map_yaml=self.root/"map.yaml",
             manifest=self.root/"safe.manifest.json")

    def tearDown(self):
        self.tmp.cleanup()

    def check(self):
        p=read_json(self.args.plan);r,out,f=analyze(p)
        maps=check_maps(p,out,self.args,r["options"],f)
        return maps,f

    def test_matching_map_and_manifest_pass(self):
        _,f=self.check();self.assertFalse(f.blocked(),f.items())

    def test_no_map_information_blocks(self):
        for k in ("source_geojson","target_geojson","map_yaml","manifest"):setattr(self.args,k,None)
        _,f=self.check();self.assertEqual(len(f.items()),4);self.assertTrue(f.blocked())

    def test_same_frame_and_bounds_different_obstacles_rejected(self):
        self.args.target_geojson=self.root/"other.geojson"
        dump(self.args.target_geojson,geometry([[[6,6],[8,6],[8,8],[6,8],[6,6]]]))
        maps,f=self.check()
        self.assertEqual(maps["source_bounds"],maps["target_bounds"])
        self.assertFalse(maps["same_obstacle_geometry"])
        self.assertIn("MAP_GEOMETRY_MISMATCH",{i["code"] for i in f.items()})
        self.assertIn("RASTER_GEOMETRY_MISMATCH",{i["code"] for i in f.items()})

    def test_missing_manifest_blocks(self):
        self.args.manifest=None
        self.assertTrue(self.check()[1].blocked())

    def test_manifest_hash_tamper_blocks(self):
        m=read_json(self.args.manifest);m["sha256"]["plan"]="bad";dump(self.args.manifest,m)
        self.assertIn("MANIFEST_HASH_MISMATCH",{i["code"] for i in self.check()[1].items()})

    def test_translation_is_rejected(self):
        m=read_json(self.args.manifest);m["source_to_target_se2"]=[1,0,0];dump(self.args.manifest,m)
        maps,f=self.check();self.assertTrue(f.blocked());self.assertFalse(maps["coordinates_transformed"])

    def test_units_degrees_rejected(self):
        m=read_json(self.args.manifest);m["units"]="degrees";dump(self.args.manifest,m)
        self.assertTrue(self.check()[1].blocked())

    def test_out_of_bounds(self):
        p=read_json(self.args.plan)
        for pt in p["segments"][0]["poses"]:pt["x"]+=20
        dump(self.args.plan,p)
        self.assertIn("RASTER_PATH_UNSAFE_OR_OUT_OF_BOUNDS",{i["code"] for i in self.check()[1].items()})

    def test_mid_edge_obstacle_not_only_endpoints(self):
        dump(self.root/"obstacle.geojson",geometry([[[1.9,.8],[2.1,.8],[2.1,1.2],[1.9,1.2],[1.9,.8]]]))
        s=[segment("cross","row",[(1,1),(3,1)])]
        self.assertTrue(geometry_failures(s,*field_geometry(self.root/"obstacle.geojson"),0))
        im=Image.open(self.root/"map.pgm");im.putpixel((30,99),0);im.save(self.root/"map.pgm")
        self.assertTrue(raster_check(s,raster(self.args.map_yaml),0))

    def test_unknown_cells_block(self):
        im=Image.open(self.root/"map.pgm");im.putpixel((30,99),205);im.save(self.root/"map.pgm")
        r=raster(self.args.map_yaml)
        self.assertEqual(r["unknown_cells"],1)
        self.assertTrue(raster_check([segment("cross","row",[(1,1),(3,1)])],r,0))

    def test_clearance_not_only_centerline(self):
        dump(self.root/"obstacle.geojson",geometry([[[1,1.1],[3,1.1],[3,2],[1,2],[1,1.1]]]))
        s=cases()["safe"]["segments"];g=field_geometry(self.root/"obstacle.geojson")
        self.assertFalse(geometry_failures(s,*g,0))
        self.assertTrue(geometry_failures(s,*g,.2))

    def test_rotated_map_origin(self):
        cfg=read_yaml(self.args.map_yaml);cfg["origin"]=[10,-1,math.pi/2]
        self.args.map_yaml.write_text(yaml.safe_dump(cfg))
        r=raster(self.args.map_yaml)
        self.assertFalse(raster_check(cases()["safe"]["segments"],r,0))

    def test_negate(self):
        cfg=read_yaml(self.args.map_yaml);cfg["negate"]=1
        self.args.map_yaml.write_text(yaml.safe_dump(cfg))
        self.assertTrue(raster_check(cases()["safe"]["segments"],raster(self.args.map_yaml),0))

    def test_bad_yaml_resolution(self):
        cfg=read_yaml(self.args.map_yaml);cfg["resolution"]=0
        self.args.map_yaml.write_text(yaml.safe_dump(cfg))
        with self.assertRaises(ValueError):raster(self.args.map_yaml)

    def test_cli_exit_codes_and_read_only(self):
        original=sha(self.args.plan)
        args=["--plan",str(self.args.plan),"--out",str(self.root/"report")]
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(main(args),1)
            for k in ("source_geojson","target_geojson","map_yaml","manifest"):
                args += ["--"+k.replace("_","-"),str(getattr(self.args,k))]
            self.assertEqual(main(args),0)
        self.assertEqual(sha(self.args.plan),original)
        result=read_json(self.root/"report/report.json")
        self.assertFalse(result["simulation_tested"])
        self.args.plan.write_text('{"segments": [NaN]}')
        with contextlib.redirect_stdout(io.StringIO()),contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(main(args),2)

    def test_upstream_drift_rejected(self):
        (self.root/"fake.py").write_text("# different source")
        with self.assertRaises(ValueError):verify_upstream(self.root/"fake.py")


def read_yaml(path):
    return yaml.safe_load(Path(path).read_text())


if __name__=="__main__":
    unittest.main()
