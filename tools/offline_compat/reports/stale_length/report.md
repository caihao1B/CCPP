# Offline CCPP → ROS2 compatibility audit

Status: **BLOCKED**

ROS 2 / Gazebo / Nav2 execution: **NOT TESTED**.

| Metric | Before | After |
|---|---:|---:|
| segment_count | 3 | 3 |
| pose_count | 9 | 9 |
| path_length_m | 4.5 | 4.5 |
| seam_length_m | 0.0 | 0.0 |
| duplicate_edges | 0 | 0 |
| discontinuous_seams | 0 | 0 |

| Finding | Origin | Count | Blocks import |
|---|---|---:|---|
| LENGTH_METADATA_MISMATCH | original_plan | 1 | True |

Full IDs, kinds, lengths, seams and source mappings are in report.json and the CSV files.
No executable plan is written and no coordinates are transformed.
