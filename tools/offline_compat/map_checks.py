"""Conservative map admission checks. No coordinate fitting or translation."""
from pathlib import Path
import math

from PIL import Image
import yaml
from shapely.affinity import affine_transform
from shapely.geometry import shape, Point, LineString, box
from shapely.ops import unary_union
from shapely.strtree import STRtree

from validate import read_json, sha, finite, xy


def field_geometry(path):
    data = read_json(path)
    if data.get("type") != "FeatureCollection":
        raise ValueError("Map must be a GeoJSON FeatureCollection")
    # This pipeline consumes local Cartesian GeoJSON, not longitude/latitude.
    if data.get("crs"):
        raise ValueError("Explicit CRS requires a separately reviewed conversion; no automatic transform")
    fields, obstacles = [], []
    for f in data.get("features", []):
        props = f.get("properties") or {}
        kind = props.get("kind", props.get("type", "")).lower()
        if kind not in ("field", "obstacle"):
            continue
        g = shape(f["geometry"])
        if g.geom_type not in ("Polygon", "MultiPolygon") or g.is_empty or not g.is_valid:
            raise ValueError("Invalid field/obstacle polygon; geometry is not auto-repaired")
        if any(not math.isfinite(v) for v in g.bounds):
            raise ValueError("Nonfinite map bounds")
        (fields if kind == "field" else obstacles).append(g)
    if len(fields) != 1:
        raise ValueError("Exactly one field feature is required")
    obstacle = unary_union(obstacles)
    return fields[0], obstacle


def primitives(segs):
    """All segments and inter-segment joins, including singleton paths."""
    previous = None
    for seg in segs:
        pts = [xy(p) for p in seg["poses"]]
        if previous is not None:
            yield dict(id=seg["id"], edge="incoming_seam"), (Point(pts[0]) if previous == pts[0]
                                                            else LineString([previous, pts[0]]))
        if len(pts) == 1:
            yield dict(id=seg["id"], edge=0), Point(pts[0])
        for j, (a,b) in enumerate(zip(pts,pts[1:])):
            yield dict(id=seg["id"], edge=j), Point(a) if a == b else LineString([a,b])
        previous = pts[-1]


def geometry_failures(segs, field, obstacles, clearance):
    safe_field = field.buffer(-clearance) if clearance else field
    unsafe_obstacles = obstacles.buffer(clearance) if clearance else obstacles
    failures = []
    for where, g in primitives(segs):
        # Touching an obstacle is conservatively rejected, including clearance boundary.
        if not safe_field.covers(g) or g.intersects(unsafe_obstacles):
            failures.append(where)
    return failures


def raster(path):
    cfg = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if cfg.get("mode", "trinary") != "trinary":
        raise ValueError("Only trinary occupancy maps are supported")
    res = cfg["resolution"]
    origin = cfg["origin"]
    if not finite(res) or res <= 0 or not isinstance(origin,list) or len(origin)!=3 or not all(finite(n) for n in origin):
        raise ValueError("Invalid map resolution/origin")
    free, occupied = cfg["free_thresh"], cfg["occupied_thresh"]
    if not all(finite(n) for n in (free,occupied)) or not 0 <= free < occupied <= 1:
        raise ValueError("Invalid occupancy thresholds")
    if type(cfg["negate"]) is not int or cfg["negate"] not in (0,1):
        raise ValueError("negate must be 0 or 1")
    image = Path(cfg["image"])
    if not image.is_absolute():
        image = Path(path).parent/image
    with Image.open(image) as im:
        if im.mode != "L":
            raise ValueError("Only 8-bit single-channel occupancy images are supported")
        width, height = im.size
        pixels = list(im.tobytes())
    # World -> raster local metric coordinates, including a rotated map origin.
    ox, oy, theta = origin
    c, s = math.cos(theta), math.sin(theta)
    inv = [c, s, -s, c, -c*ox-s*oy, s*ox-c*oy]
    local_bounds = box(0,0,width*res,height*res)
    blocked, unknown, occupied_count = [], 0, 0
    for row in range(height):
        for col in range(width):
            v = pixels[row*width+col]/255
            probability = v if cfg["negate"] else 1-v
            is_free = probability < free
            if not is_free:
                unknown += probability <= occupied
                occupied_count += probability > occupied
                y = (height-row-1)*res
                blocked.append(box(col*res,y,(col+1)*res,y+res))
    return dict(cfg=cfg, image=image, width=width,height=height,resolution=res,
                inverse=inv, bounds=local_bounds, blocked=blocked, tree=STRtree(blocked),
                unknown_cells=unknown, occupied_cells=occupied_count, pixels=pixels)


def raster_check(segs, r, clearance):
    failures=[]
    for where,g in primitives(segs):
        local = affine_transform(g,r["inverse"])
        # distance() tests the whole line, never just its sampled endpoints.
        unsafe = not r["bounds"].covers(local) or (clearance > 0 and
                 local.distance(r["bounds"].boundary) <= clearance)
        candidates = r["tree"].query(local.envelope.buffer(clearance).envelope if clearance else local.envelope)
        if any(local.distance(r["blocked"][int(i)]) <= clearance for i in candidates):
            unsafe = True
        if unsafe:
            failures.append(where)
    return failures


def raster_alignment(field, obstacles, r):
    """Full-image consistency away from a one-pixel diagonal boundary band.

    Rasterization at polygon edges differs across renderers. That band is ignored
    for identity checking only, NOT for path collision/range checks.
    """
    f = affine_transform(field,r["inverse"])
    o = affine_transform(obstacles,r["inverse"])
    boundary = unary_union([f.boundary, o.boundary]) if not o.is_empty else f.boundary
    tol = math.sqrt(2)*r["resolution"]
    mismatches = 0
    for row in range(r["height"]):
        for col in range(r["width"]):
            pt = Point((col+.5)*r["resolution"], (r["height"]-row-.5)*r["resolution"])
            if boundary.distance(pt) <= tol:
                continue
            v = r["pixels"][row*r["width"]+col]/255
            occ = v if r["cfg"]["negate"] else 1-v
            image_free = occ < r["cfg"]["free_thresh"]
            geometry_free = f.covers(pt) and not o.covers(pt)
            mismatches += image_free != geometry_free
    return dict(mismatched_cells=mismatches, boundary_band_m=tol)


def check_maps(plan, after, args, options, findings):
    result = dict(coordinates_transformed=False, simulation_tested=False)
    source = target = None
    if args.source_geojson:
        source = field_geometry(args.source_geojson)
        result["source_bounds"] = list(source[0].bounds)
        for where in geometry_failures(plan["segments"], *source, options["clearance_m"]):
            findings.add("SOURCE_GEOMETRY_UNSAFE", "original_plan", where)
        # Includes all original seams, so collisions inherited from a seam are not
        # falsely called a newly invented geometric route.
        result["source_unsafe_after_count"] = len(geometry_failures(after, *source, options["clearance_m"]))
    else:
        findings.add("SOURCE_MAP_REQUIRED", "map_admission", "Provide original planning geometry")
    if args.target_geojson:
        target = field_geometry(args.target_geojson)
        result["target_bounds"] = list(target[0].bounds)
        for where in geometry_failures(after, *target, options["clearance_m"]):
            findings.add("TARGET_GEOMETRY_UNSAFE", "map_admission", where)
    else:
        findings.add("TARGET_MAP_REQUIRED", "map_admission", "Provide intended ROS/Gazebo geometry")
    if source and target:
        result["same_field_geometry"] = source[0].equals(target[0])
        result["same_obstacle_geometry"] = source[1].equals(target[1])
        if not all(result[k] for k in ("same_field_geometry", "same_obstacle_geometry")):
            findings.add("MAP_GEOMETRY_MISMATCH", "map_admission", "Boundary and obstacle geometry must both agree; no auto-translation")
    r = None
    if args.map_yaml:
        r = raster(args.map_yaml)
        result["raster"] = dict(width=r["width"],height=r["height"],resolution=r["resolution"],
            origin=r["cfg"]["origin"], image_sha256=sha(r["image"]),
            unknown_cells=r["unknown_cells"], occupied_cells=r["occupied_cells"])
        for where in raster_check(after,r,options["clearance_m"]):
            findings.add("RASTER_PATH_UNSAFE_OR_OUT_OF_BOUNDS", "map_admission", where)
        if target:
            result["raster_alignment"] = raster_alignment(*target,r)
            if result["raster_alignment"]["mismatched_cells"]:
                findings.add("RASTER_GEOMETRY_MISMATCH", "map_admission", result["raster_alignment"])
            if not r["bounds"].covers(affine_transform(target[0],r["inverse"])):
                findings.add("TARGET_FIELD_OUTSIDE_RASTER", "map_admission", "Target field exceeds raster extent")
    else:
        findings.add("RASTER_REQUIRED", "map_admission", "Map YAML plus its image are required")
    if not args.manifest:
        findings.add("COORDINATE_EVIDENCE_REQUIRED", "map_admission", "A reviewed hash-bound coordinate manifest is required")
    else:
        m = read_json(args.manifest)
        if (m.get("version") != 1 or m.get("units") != "m" or m.get("axes") != "x_y_right_handed"
            or m.get("frame_id") != plan["frame_id"] or m.get("source_to_target_se2") != [0,0,0]
            or not isinstance(m.get("coordinate_review"),str) or not m["coordinate_review"].strip()):
            findings.add("COORDINATE_MANIFEST_INVALID", "map_admission", "Require metres, matching frame, explicit identity transform and review rationale")
        files = dict(plan=args.plan, source_geojson=args.source_geojson,
                     target_geojson=args.target_geojson,map_yaml=args.map_yaml,
                     map_image=r["image"] if r else None)
        hashes = m.get("sha256", {})
        if not isinstance(hashes,dict):
            hashes = {}
        for k,path in files.items():
            if path is None or hashes.get(k) != sha(path):
                findings.add("MANIFEST_HASH_MISMATCH", "map_admission", k)
    result["limit"] = "2D geometry/raster checks; manifest records human coordinate review. Does not verify Gazebo SDF, TF, kinematics, controller or footprint sweep."
    return result
