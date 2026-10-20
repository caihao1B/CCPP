# Offline CCPP → ROS2 compatibility audit

Status: **BLOCKED**

ROS 2 / Gazebo / Nav2 execution: **NOT TESTED**.

| Metric | Before | After |
|---|---:|---:|
| segment_count | 3 | 2 |
| pose_count | 9 | 9 |
| path_length_m | 4.5 | 5.5 |
| seam_length_m | 2.0 | 1.0 |
| duplicate_edges | 0 | 0 |
| discontinuous_seams | 2 | 1 |

| Finding | Origin | Count | Blocks import |
|---|---|---:|---|
| INPUT_SEAM_GAP | original_plan | 2 | True |
| KIND_SEMANTICS_LOST | executor_merge | 1 | True |
| SEAM_PROMOTED_TO_INTERNAL_EDGE | executor_merge | 1 | True |
| LENGTH_METADATA_MISMATCH | executor_merge | 1 | True |

Full IDs, kinds, lengths, seams and source mappings are in report.json and the CSV files.
No executable plan is written and no coordinates are transformed.
