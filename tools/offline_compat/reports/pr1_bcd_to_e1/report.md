# Offline CCPP → ROS2 compatibility audit

Status: **BLOCKED**

ROS 2 / Gazebo / Nav2 execution: **NOT TESTED**.

| Metric | Before | After |
|---|---:|---:|
| segment_count | 3337 | 1899 |
| pose_count | 31377 | 29939 |
| path_length_m | 20914.120608274265 | 20914.120608274265 |
| seam_length_m | 0.0 | 0.0 |
| duplicate_edges | 0 | 0 |
| discontinuous_seams | 0 | 0 |

| Finding | Origin | Count | Blocks import |
|---|---|---:|---|
| OVERSIZED_INPUT_EDGE | original_plan | 14 | True |
| SEGMENT_TOO_LONG | original_plan | 1 | True |
| KIND_SEMANTICS_LOST | executor_merge | 1296 | True |
| JUNCTION_ORIENTATION_DROPPED | executor_merge | 1438 | True |
| LENGTH_METADATA_MISMATCH | executor_merge | 1164 | True |
| TARGET_GEOMETRY_UNSAFE | map_admission | 29877 | True |
| MAP_GEOMETRY_MISMATCH | map_admission | 1 | True |
| RASTER_PATH_UNSAFE_OR_OUT_OF_BOUNDS | map_admission | 29875 | True |
| COORDINATE_EVIDENCE_REQUIRED | map_admission | 1 | True |

Full IDs, kinds, lengths, seams and source mappings are in report.json and the CSV files.
No executable plan is written and no coordinates are transformed.
