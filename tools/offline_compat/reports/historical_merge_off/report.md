# Offline CCPP → ROS2 compatibility audit

Status: **BLOCKED**

ROS 2 / Gazebo / Nav2 execution: **NOT TESTED**.

| Metric | Before | After |
|---|---:|---:|
| segment_count | 101 | 101 |
| pose_count | 1737 | 1737 |
| path_length_m | 1633.7700962091383 | 1633.7700962091383 |
| seam_length_m | 0.0 | 0.0 |
| duplicate_edges | 0 | 0 |
| discontinuous_seams | 0 | 0 |

| Finding | Origin | Count | Blocks import |
|---|---|---:|---|
| SOURCE_GEOMETRY_UNSAFE | original_plan | 62 | True |
| TARGET_GEOMETRY_UNSAFE | map_admission | 1736 | True |
| MAP_GEOMETRY_MISMATCH | map_admission | 1 | True |
| RASTER_PATH_UNSAFE_OR_OUT_OF_BOUNDS | map_admission | 1736 | True |
| COORDINATE_EVIDENCE_REQUIRED | map_admission | 1 | True |

Full IDs, kinds, lengths, seams and source mappings are in report.json and the CSV files.
No executable plan is written and no coordinates are transformed.
