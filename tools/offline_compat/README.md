# CCPP → ROS2_BT- 独立离线兼容性验证

本工具只读输入，不连接 ROS 2、不发送 FollowPath、不修改 Plan、Humble YAML 或历史实验数据。输出是审计报告，不是可执行路径。**OFFLINE_PASS 不等于已完成 Gazebo/Nav2 仿真验证。**

## 1. 已复核的版本和执行范围

| 对象 | 固定提交 | 作用 |
|---|---|---|
| CCPP main | `c59ef20dc986fa8df50837ca34361a2dbb4a0b09` | Plan 数据类、历史样本、原始农田 |
| CCPP PR #1 | `8fa580bb53ed14bce4b87422ac38b569650fcb61` | **尚未合并**的连接安全修复，用于重新生成 BCD 样本 |
| ROS2_BT- main | `f523316be5e83863b27a52507708648a1d752a1f` | `plan_io.py`、`executor_node.py`、E1 profile、地图 |

`legacy_merge.py` 是固定版本 `plan_io.py` 中三个纯函数的原样摘录，保留已发现的行为，不能在此偷偷修正。`--ros-plan-io` 同时核对原文件 SHA-256 和三个函数的 AST；版本漂移时退出 2，要求重新审计。无需 ROS 消息包或运行中的节点。

**范围特别说明：**复现的是 `coverage_manager/executor_node.py` 的 `coverage_executor` 路径。`coverage_bt_supervisor/.../supervisor_node.py::_load_plan()` 独立读取 JSON，并未调用此函数；`scripts/e1_run_t3_method.sh` 的 `nav2_base` 默认可执行文件是 `coverage_executor_nobt_rejoin`，BT 分支也有独立入口。因此不能把本工具的合并结果推广成所有 E1、BT 或历史论文实验的实际运行结果。实际启动命令、安装空间中的代码及最终参数仍须核对。

## 2. 接口复核

| 字段 | CCPP 输出 | coverage_executor 的读取/处理 | 本工具 |
|---|---|---|---|
| `frame_id` | Plan / Segment / Pose 均可有，默认 `map` | `segment_to_path` 用段或计划 frame；忽略 pose frame。执行器默认 `force_plan_frame_id=true`，直接覆盖标签，没有坐标变换 | 拒绝三层 frame 冲突；同名 frame 不作为同地图证据 |
| `segments[].id` | 字符串 | 合并保留前段 ID；状态存储和事件使用合并后 ID | 拒绝空/重复 ID，输出完整原段→执行段对应关系 |
| `kind` | `intra_cluster/bridge/row/turn/patch` | 合并保留前段 kind，后段类型不再独立发布 | 跨类型合并阻断，记录消失的事件 ID |
| `poses[].x/y` | 米制平面位置的预期用途 | `_safe_float` 允许转换/默认值 | 只接受有限 JSON 数字，不接受字符串、布尔、NaN、Inf |
| `yaw, qx/qy/qz/qw` | 通常两者都有 | 完整四元数优先，否则由 yaw 计算；不主动归一化 | 检查完整性、单位范数、平面姿态、yaw 一致性；支持 q 与 −q 等价；yaw-only 单独标记 |
| `z` | Pose2D 无此字段 | 默认 0 | 非零或非法 z 阻断 |
| `cluster_id` | 整数 | 此合并函数不保留被吸收段的 cluster_id | 校验类型，在来源映射中保留原段信息；并不模拟 BT 的 cluster 逻辑 |
| `meta` | 任意 JSON 对象，不保证有 length_m | `length_m` 优先决定是否短段；写 merged/merged_from/length_m，保留前段其他值 | 同时报告几何长度和声明长度，不替输入补写或纠正 |
| `field_meta` | 来源路径、算法参数等 | 此执行器不据此验证地图 | 原样记录；来源字符串不等于地图坐标证明 |
| `clusters/cluster_order/transitions` | 列表结构 | 此执行器不重映射这些顶层数据 | 校验基本结构；transitions 非空且发生合并时要求人工复核 |

缺失非执行必需的 CCPP 顶层字段会留下提示；非法结构会拒绝。未知附加字段保留在原文件中。工具对输入比 ROS 的宽松读取器更严格，以避免把非法数值默认为 0 或把 frame 重标记误认为真实变换。yaw-only 是 coverage_executor 的兼容形式，不代表 BT Supervisor 也采用相同回退逻辑。

## 3. 合并规则及发现

E1 profile：`merge_short_segments=true`、`merge_length_threshold_m=2.0`、`merge_min_poses=3`。实际条件是 **length < 2.0 OR poses < 3**；不是 AND，也不是 <=。首段无前段可合并，仍被保留。因此两点长路径也会被合并，而首段单点/零长度不会自动消失。

1. 前段 ID、kind 及其他元数据保留。`row + turn` 会变为一个 row；`turn + row` 也可能把作业行藏在 turn 中。报告保留每个来源的 ID、kind、meta 和顺序，不信任输入中的 merged_from 来推导来源。
2. 仅在接点 x/y **完全相等**时删除后段首点，不比较 yaw 或四元数。同位置的不同朝向可能被删除；相差 1e-9 m 的点却会留下。报告区分原有重复点、正常接点去重、合并后留下的近重复点和被删朝向。
3. 不检查接缝距离。输入原有间隙被合并后，会成为一个 FollowPath 内部的直线边。报告同时记录 `original_plan/INPUT_SEAM_GAP` 与 `executor_merge/SEAM_PROMOTED_TO_INTERNAL_EDGE`：几何间隙来源于原计划，隐式段内连接来源于预处理。
4. 前段缺少 `meta.length_m` 时，函数先追加 poses，再计算已合并长度，再加当前段长度，造成重复计长。若前段已有 length_m，则使用声明长度相加，不补入接缝距离；陈旧长度也可能改变是否合并。
5. 合并会改变 segment_start 的候选 ID/kind 和完成状态粒度。报告不伪造 ROS 运行事件；未证明现有统计已经出错。原始 JSON 的 SHA 也不能区分不同合并参数产生的段索引，后续新实验必须使用新的状态文件/日志，关闭旧状态恢复。

默认诊断限值：段间连续性 1e-6 m、近重复点 1e-6 m、单边最大长度 2 m、单段最大长度 100 m。后两者是可调的审查阈值，**不是 Nav2 的普适物理上限**。较长直边不等于穿障碍或规划失败，但需要检查采样密度和执行适应性。几何 `clearance_m` 默认 0，仅表示中心线检查。

## 4. 已执行的离线结果

测试环境：Python 3.12.14、Shapely 2.1.2 / GEOS 3.13.1、PyYAML 6.0.3、Pillow 11.3.0、NumPy 2.3.5；重生成 BCD 另用 NetworkX 3.5。

| 样本，E1 合并参数 | 段数量 | row | turn | 合并前后总几何长度 |
|---|---:|---:|---:|---:|
| main 历史 `plan_boustrophedon.json` | 101 → 83 | 51 → 45 | 50 → 38 | 1633.770096 m，不变 |
| PR #1 固定提交重生成 BCD | 3337 → 1899 | 1669 → 1490 | 1668 → 409 | 20914.120608 m，不变 |

| 检查 | 历史样本 | PR #1 BCD |
|---|---:|---:|
| 被吸收的段 / 删除的相同 XY 接点 | 18 / 18 | 1438 / 1438 |
| 跨 kind 合并 | 15 | 1296 |
| 被删除且朝向不同的接点 | 5 | 1438 |
| 合并后长度元数据不一致的段 | 14 | 1164 |
| 原始不连续接缝 / 合并后不连续接缝 | 0 / 0 | 0 / 0 |
| 原始段内重复点 / 合并后段内重复点 | 0 / 0 | 0 / 0 |
| 原地图中心线不安全边（合并前/后） | 62 / 62 | 0 / 0 |

这证明本次真实样本中的合并改变了语义和姿态接点，没有凭空改变二维折线路线。历史样本的 62 条穿障碍边属于原规划结果；合并后依旧存在。BCD 的零穿障碍结论仅适用于此输入、原农田、二维中心线检查，不代表机器人可执行。BCD 还有 14 个原段存在超过默认 2 m 的采样边，最大边约 22.758 m；1 个原段超过默认 100 m，约 145.1 m。这些输入问题与合并变化分别报告。

农田边界为 `[1.625, 4.625, 38.375, 28.375]`，即 36.75 × 23.75 m；所测 E1 `low_01_5p0m_scatter` 为 `[0,0,5,5]`，其栅格含边缘留白，为 140 × 140 像素、0.05 m/像素、origin `[-1,-1,0]`。field 与 raster 大小不能混为一谈。两套几何不一致、路径越界，且没有已审核坐标清单，故明确拒绝直接导入。

35 个 unittest 测试方法通过，包括 20 个非法输入子用例、60 组固定随机种子的轨迹/映射守恒验证。另有 10 个 CLI 场景：安全合成样本及关闭合并的合成短 turn 为预期 OFFLINE_PASS，其余为预期 BLOCKED。详见 `reports/summary.json`、逐段 CSV、JSON 及 `reports/tests.log`。

## 5. 安装与完整运行命令

工具本身支持 Python 3.10+，推荐在独立 Python 3.12 环境运行。ROS 2 Humble 是否安装/启动不影响它。以下从本分支的 CCPP 根目录执行；PowerShell 中也可使用 `python`，激活命令见下方。

```bash
python -m venv .venv-compat
source .venv-compat/bin/activate
python -m pip install -r tools/offline_compat/requirements.txt
python -m unittest discover -s tools/offline_compat/tests -v
python tools/offline_compat/run_suite.py --out tools/offline_compat/results/synthetic
```

Windows PowerShell 激活：`.\.venv-compat\Scripts\Activate.ps1`。若不想修改执行策略，直接把命令中的 `python` 替换为 `.\.venv-compat\Scripts\python.exe`。

获取独立的 ROS 源码副本（不改正在使用的工作空间；禁用换行转换以便校验原始文件哈希）：

```bash
git -c core.autocrlf=false clone https://github.com/caihao1B/ROS2_BT-.git ../ROS2_BT-audit
git -C ../ROS2_BT-audit checkout --detach f523316be5e83863b27a52507708648a1d752a1f
python tools/offline_compat/run_suite.py --out tools/offline_compat/results/full --ccpp-root . --ros-root ../ROS2_BT-audit
```

单独检查历史样本（该命令**预期退出 1**）：

```bash
python tools/offline_compat/validate.py --plan plan_boustrophedon.json --profile ../ROS2_BT-audit/profiles/executor_e1_static.yaml --ros-plan-io ../ROS2_BT-audit/src/coverage_manager/coverage_manager/plan_io.py --source-geojson field.geojson --target-geojson ../ROS2_BT-audit/datasets/raw/maps_30/geojson/low_01_5p0m_scatter.geojson --map-yaml ../ROS2_BT-audit/e1_assets/maps/low_01_5p0m_scatter.yaml --out tools/offline_compat/results/historical
```

单独检查确定可通过的**合成**样本：

```bash
python tools/offline_compat/validate.py --plan tools/offline_compat/examples/safe.json --source-geojson tools/offline_compat/examples/field.geojson --target-geojson tools/offline_compat/examples/field.geojson --map-yaml tools/offline_compat/examples/map.yaml --manifest tools/offline_compat/examples/safe.manifest.json --out tools/offline_compat/results/safe
```

重现 PR #1 BCD（可选；建议 Python 3.12；不会合并 PR 或改 main）：

```bash
git fetch origin codex/fix-safe-transitions-20261017
git worktree add --detach ../CCPP-pr1-audit 8fa580bb53ed14bce4b87422ac38b569650fcb61
python -m pip install networkx==3.5 numpy==2.3.5
python -m pip install --no-deps -e ../CCPP-pr1-audit
python -B -m fieldplan_offline.cli.plan_boustrophedon --geojson field.geojson --method bcd --spacing 1 --margin 0.2 --step 1 --connect_cell 0.5 --out ../pr1_bcd_generated.json
python tools/offline_compat/run_suite.py --out tools/offline_compat/results/with_pr1 --ccpp-root . --ros-root ../ROS2_BT-audit --pr1-plan ../pr1_bcd_generated.json
```

`run_suite.py` 检查各场景的预期退出码，正常完成返回 0；单次 `validate.py` 返回 1 不代表工具崩溃。PR #1 输出从未被当作 main 的规划结果。跨不同几何库版本生成的浮点细节可能变化，应保存依赖版本及输入哈希。

## 6. 报告与退出码

| 退出码 | 状态 | 意义 |
|---|---|---|
| 0 | OFFLINE_PASS | 提供的输入通过此次离线门槛；尚未仿真 |
| 1 | BLOCKED | 完成分析但发现语义/几何问题，或地图证据不齐；拒绝直接导入 |
| 2 | INVALID_INPUT | 格式/配置非法、文件不存在、源代码漂移等；不能完成可靠分析 |

- `report.json`：输入 SHA-256、参数、固定源版本、全部 before/after 段、所有接缝、每次合并、所有事件消失 ID、来源映射、地图结果。每类 finding 保留精确计数和最多 10 个示例；段和合并明细不截断。仓库内 BCD 的大报告以 `report.json.gz` 无损保存，运行工具仍直接生成 JSON；可用 `python -c "import gzip,pathlib;pathlib.Path('bcd_report.json').write_bytes(gzip.open('tools/offline_compat/reports/pr1_bcd_to_e1/report.json.gz','rb').read())"` 解压。
- `before_segments.csv` / `after_segments.csv`：ID、kind、点数、几何/声明长度、最大边、重复点计数，可直接表格查看。
- `report.md`：简短概览；`summary.json`：整套命令的结果与环境。命令输出目录可覆盖自己的报告，应为每轮实验使用新的目录。
- `--merge off`：仅做对照分析，不会修改 Humble profile；不会自动替你更改实际启动参数。

## 7. 地图准入规则与坐标清单

必须同时提供原规划 GeoJSON、目标 GeoJSON、目标 map YAML+image、已审核坐标 manifest。缺任一项仍能观察合并行为，但最终拒绝导入。

检查顺序：

1. 计划内各层 frame 一致，位置/四元数有效。
2. 原/目标农田边界及障碍物并集在**原坐标**下几何相等。只包围盒相等、frame 都叫 map 或文件名相同均不足。
3. 全部路径边和接缝位于原/目标可行域，整条线段检查，不仅检查点。障碍边界接触保守判为不安全。
4. YAML 的 resolution、origin（含 yaw）、negate、阈值合法；支持 8 位灰度 trinary 地图。正确处理图片上下轴方向；unknown 一律不可通行。旋转 origin 仅用于地图索引转换，不改变任何计划坐标。
5. 整条线段不越栅格边界、不穿 occupied/unknown 栅格。设置 `--clearance-m` 后检查与整条线及栅格的距离。此处没有把疏采样边当作安全。
6. 对整幅 raster 与目标 GeoJSON 的空闲/占用关系进行比对。身份比对在几何边缘忽略一个像素对角线宽度的栅格化差异带；**路径碰撞检查不忽略该带**。这不是 Gazebo 模型验证。
7. Manifest 绑定计划、两份 GeoJSON、YAML、image 的实际字节哈希，并明确米制、右手 XY、相同 frame、identity SE(2)。非 identity 变换直接拒绝，不做平移/旋转拟合。

实际清单模板（将占位值换为相应文件 `sha256`，并写明真实坐标审核依据；不能照抄 synthetic 的说明充当真实证据）：

```json
{
  "version": 1,
  "units": "m",
  "axes": "x_y_right_handed",
  "frame_id": "map",
  "source_to_target_se2": [0, 0, 0],
  "coordinate_review": "说明来源坐标原点、轴向、尺度及目标地图生成依据",
  "sha256": {
    "plan": "<sha256>",
    "source_geojson": "<sha256>",
    "target_geojson": "<sha256>",
    "map_yaml": "<sha256>",
    "map_image": "<sha256>"
  }
}
```

生成单文件哈希：

```bash
python -c "import hashlib,pathlib;print(hashlib.sha256(pathlib.Path('field.geojson').read_bytes()).hexdigest())"
```

manifest 是可追溯的人工坐标审核记录，**不是证明地理真实性的密码学签名**。路径来源、地图制作过程仍需要实际核实。GeoJSON 如带 CRS 声明，本工具保守拒绝自动转换；经纬度数据须先在独立流程中正确投影并同步生成地图、障碍和 Gazebo 模型。不存在“只平移路径即可”的通过选项。

## 8. 何时进入 Gazebo/Nav2 执行验证

下列条件满足后，才适合开新的、小规模仿真实验：

1. 固定规划代码/参数/输入哈希；选择经过几何检查的新结果。历史碰撞样本不能因字段兼容而投入执行。
2. 确认实际执行入口及最终加载的 Humble 参数。若仍用被审计的 coverage_executor，应针对新实验明确选择保留语义的预处理方案，或使用独立参数覆盖关闭合并，并重新评估短段切换。不能只在校验命令里 `--merge off`，实际运行仍开启合并。
3. 解决或明确审核所有阻断项：跨类型合并、朝向丢失、长度元数据、断接、超长边/段。没有“忽略所有错误”的开关，也不在本补丁中静默修复路径。
4. 计划、原/目标 GeoJSON、栅格坐标和障碍全部一致；另行检查 Gazebo `.world/.sdf` collision 几何、出生点和墙体位置。工具未解析 SDF，也不验证 `/map → odom → base_link` 的实际 TF。
5. 用真实车体外接半径及安全裕量选择 clearance，检查 footprint、转弯半径、前进/倒车与原地旋转策略、速度/加速度约束。中心线通过不能替代车体扫掠或运动学验证；姿态与切向关系也需要控制器针对性检查。
6. 设置新的状态文件、日志和输出目录；根据实际执行器关闭旧状态恢复。先静态、低速、少量连续 `row → turn → row`，再扩大范围；若启用 smoother，必须重新检查实际平滑路径。
7. 比较实际接收的 Path、segment_start ID/kind、动作结果、TF、costmap、碰撞/最小间距、轨迹跟踪误差、覆盖率、耗时与 A/C/D 事件。动态障碍实验排在静态执行确认之后。

本次工作到“离线接口/预处理/地图准入验证”结束，未运行 ROS 2、Gazebo、Nav2，也未重新解释或改写历史论文结果。
