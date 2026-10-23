# Offline CCPP → ROS2 compatibility audit

Status: **BLOCKED**

ROS 2 / Gazebo / Nav2 execution: **NOT TESTED**.

| Metric | Before | After |
|---|---:|---:|
| segment_count | 3 | 2 |
| pose_count | 9 | 9 |
| path_length_m | 4.5 | 4.500000001 |
| seam_length_m | 1.000000082740371e-09 | 0.0 |
| duplicate_edges | 0 | 0 |
| discontinuous_seams | 0 | 0 |

| Finding | Origin | Count | Blocks import |
|---|---|---:|---|
| KIND_SEMANTICS_LOST | executor_merge | 1 | True |
| NEAR_DUPLICATE_JUNCTION_RETAINED | executor_merge | 1 | False |
| LENGTH_METADATA_MISMATCH | executor_merge | 1 | True |

Full IDs, kinds, lengths, seams and source mappings are in report.json and the CSV files.
No executable plan is written and no coordinates are transformed.
