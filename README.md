# fieldplan_offline

Offline coverage planning toolkit to generate **segment-based paths** (Plan JSON) that can be executed in ROS2/Nav2 using **FollowPath**.

## What you get
- Square-grid decomposition (optional blocked cells kept)
- Clustering + cluster-level TSP ordering
- Intra-cluster planners (TSP-GA/Christofides, MST-Euler)
- Stitching to ensure graph connectivity (penalized shortest paths)
- Post-processing: segmentation, downsampling, headings (yaw/quaternion)
- Export: `plan.json` (segments + hierarchy), optional flattened `path.csv`
- Optional continuous boustrophedon (serpentine/swath) planner (non-grid)

## Quickstart

```bash
pip install -e .
fieldplan-plan-grid --geojson /path/to/field.geojson --cell 0.5 --out plan.json
fieldplan-visualize-plan --plan plan.json
```

GeoJSON features should include a Polygon for the field boundary and optional obstacle polygons. Supported properties:
- `properties.kind` == "field" / "obstacle"
- or `properties.type` == "field" / "obstacle"

## Output
`plan.json` contains:
- clusters (cell ids)
- cluster order
- segments: each segment has (id, kind, cluster_id, poses[x,y,yaw,qx,qy,qz,qw])

ROS2 side should read `plan.json`, convert poses to `nav_msgs/Path`, and call Nav2 FollowPath per segment.
