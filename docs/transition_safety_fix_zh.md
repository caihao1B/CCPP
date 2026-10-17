# 全覆盖规划转场路径几何安全修复

检查日期：2026-10-17。基准：`caihao1B/CCPP` 的 `c59ef20dc986fa8df50837ca34361a2dbb4a0b09`。

本补丁复用现有网格 A*，修复 continuous / BCD 的转场与接缝检查。它验证的是带配置余量的二维点路径，不是车辆运动学或实际执行安全认证。未运行 ROS 2、Nav2、仿真或实车。

## 复核与结果

历史 `plan_boustrophedon.json` 保留原样，作为已知失败样本。独立验证脚本逐一检查完整相邻点线段、全部路径段以及段间隐式接缝，不依赖规划器自己的检查函数。

比较参数：`--spacing 1 --margin 0.2 --step 1`；continuous 使用默认 `--connect straight --turn_step 1`；BCD 使用默认 DFS、自动扫描角、0.5 m 连接网格。历史文件仅记录 spacing=1、margin=0.2 等有限元数据，没有完整命令与依赖版本；此次重新运行不能当作历史生成过程的完全重现。

| 数据 | 总段数 | row / turn | 穿原始障碍的 turn 段 | 穿原始障碍的段内边 | 违反带余量可行域的段内边 | 违反可行域的接缝 | 断接 |
|---|---:|---:|---:|---:|---:|---:|---:|
| 仓库历史示例 | 101 | 51 / 50 | 23 | 62 | 83 | 0 | 0 |
| 基准代码重新生成 continuous | 103 | 52 / 51 | 22 | 55 | 69 | 0 | 0 |
| 修复后 continuous | 103 | 52 / 51 | 0 | 0 | 0 | 0 | 0 |
| 基准代码重新生成 BCD | 3332 | 1669 / 1663 | 0 | 0 | 4 | 27 | 29 |
| 修复后 BCD | 3337 | 1669 / 1668 | 0 | 0 | 0 | 0 | 0 |

- 17 个 `kind=obstacle` 多边形；历史 `seg_0005` 确实穿过 `obstacle_id=0`。
- “原始障碍相交”采用 `intersects`，包含边界接触；本表 margin=0.2 的碰撞边也进入障碍内部，不只是接触。
- “可行域”是 `field.buffer(-margin) - union(obstacle.buffer(margin))`；带余量边界允许接触。数值误差容限固定为 `1e-8 m`，不随网格尺寸或 BCD 邻接容限放大。
- 断接是相邻段的前末点与后首点相距超过 `1e-8 m`。即使接缝未显示为 turn，也必须检查它，CSV 扁平化会连接这些点。
- BCD 的 `row` 实际包含一个单元的整条覆盖折线及内部转场；仅检查 `kind=turn` 会漏检。
- continuous 的所有覆盖行 poses 在前后结果中完全相同；BCD 覆盖单元 ID 及遍历顺序保持相同，修复改变连接与可能的行方向选择。
- 含接缝总折线长度：continuous 1634.520 → 1739.340 m；BCD 21909.898 → 20914.121 m。前者增加避障绕行，后者增加安全直连优先。基准路径存在违规，长度变化不代表规划最优性结论。
- BCD 的大量细分单元及长路径是既有分解/排序问题，本补丁不重构覆盖算法，也不宣称验证了覆盖完整率。

机器可读摘要及文件 SHA-256：`docs/transition_safety_results.json`。

## 具体根因与最小改动

1. **continuous CLI** 的 `_straight_connector()` 仅插值，不知道障碍物。现在先验证完整直线；不安全时按需构建并复用 BCD 的连接网格，用 A* 绕行。逐边加密保留全部拐点，避免跨角重采样再次切穿障碍。
2. **BCD 端点接入** 原来选择欧氏最近网格点；若真实端点到网格点的线段不安全，会直接省略端点。单元内部调用随后又跳过 `conn_pts[0]`，可能引入不安全捷径；单元之间则留下断接。现在只选择有完整可见连接的网格点，必须保留准确端点，否则抛出 `NoSafePathError`。
3. **BCD 无连接器回退** 原来在简单连接失败后返回 `[a,b]`；现在明确失败，不再生成未经检查的直线。
4. **网格完整边** 原来主要检查网格中心。凹边界的窄缺口可能落在两个合法中心之间。现在检查每条图边，删除越界边，A* 的代价和启发式统一使用米。
5. **连通分量** 原网格及 BCD 连接器会只保留最大分量。新增 `prune_isolated=False` 供连接器使用，保留全部分量，禁止把小区域默默投影到大区域。其他网格规划入口默认行为不变。
6. **输出前校验** 两种模式均检查 row 内部、turn 内部及段间接缝，BCD 还在旋转回世界坐标后检查。找不到安全连接就以非零退出码失败；在构建阶段失败不会写入/覆盖 JSON、CSV。
7. **容差** 邻接容差只应影响单元邻接，不能作为穿障碍的许可。安全检查独立限制在 `1e-8 m` 数值误差范围。

BCD TSP 中的欧氏距离回退只是单元排序的估计代价，不是输出路径；最终连接仍必须通过相同的 A* 和完整线段检查。

源码改动限于现有 CLI、两个 boustrophedon 文件、网格构建可选参数，以及一个小型 `io/path_safety.py` 校验模块。没有增加运行时依赖，Plan/Segment/Pose 的 JSON 结构和 CSV 列保持不变。

`--connect straight` 参数名保留，含义调整为“优先安全直线，受阻时 A*”。`--connect_cell` 现在也控制 continuous 的连接网格。`--connect none` 保留独立覆盖行输出；它不承诺行间连续，不能把其 CSV 无条件当作一条可直接执行的路径。

## 测试与运行

验证环境：Python 3.12.14、Shapely 2.2.0 / GEOS 3.14.1、NumPy 2.5.4、NetworkX 3.7、pytest 9.1.1。共 **35 项测试通过**（包括原有导入测试），`git diff --check` 通过。测试覆盖：

- 两种 CLI 模式下的直线可通行、障碍绕行、狭窄通道、凹多边形，以及 JSON/CSV 一致性；
- 端点合法但线段穿障碍、凹缺口穿越图边、最近节点不可见、端点无法接入；
- 不连通区域及小分量保留、粗网格无法表示窄通道、空网格、零长度连接；
- BCD 缺少连接器时拒绝危险回退、DFS/TSP 不可达时失败、旋转与余量；
- 仓库真实场景在 margin=0 和 margin=0.2 下的两种模式；
- 历史样本的 23/62 复核、独立脚本检查段内边和段间接缝；
- 不可达时进程失败且不产生 JSON/CSV，保留显式 `--connect none` 行为。

在补丁分支根目录，Linux / Bash：

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install -e . pytest
export PYTHONDONTWRITEBYTECODE=1
python -m pytest -q
mkdir -p out

python -m fieldplan_offline.cli.plan_boustrophedon \
  --geojson field.geojson --method continuous --spacing 1 --margin 0.2 --step 1 \
  --out out/continuous.json --out_csv out/continuous.csv
python scripts/validate_plan_collisions.py \
  --geojson field.geojson --plan out/continuous.json --out out/continuous.audit.json

python -m fieldplan_offline.cli.plan_boustrophedon \
  --geojson field.geojson --method bcd --spacing 1 --margin 0.2 --step 1 \
  --out out/bcd.json --out_csv out/bcd.csv
python scripts/validate_plan_collisions.py \
  --geojson field.geojson --plan out/bcd.json --out out/bcd.audit.json
```

Windows PowerShell 可用 `.venv\Scripts\Activate.ps1` 激活环境，环境变量用 `$env:PYTHONDONTWRITEBYTECODE="1"`，上述 Python 命令合并为单行运行。

重新检查历史文件（**预期退出码 1**）：

```bash
python scripts/validate_plan_collisions.py \
  --geojson field.geojson --plan plan_boustrophedon.json --out out/historical.audit.json
```

验证脚本：退出码 0=完整路径位于配置可行域，1=几何违规，2=输入错误。`--margin` 不提供时从 Plan 元数据读取；需要只按原始几何检查时可显式指定 `--margin 0`。报告区分原始障碍接触、进入内部、带余量可行域违规、断接。margin=0 时可能沿障碍边界走，这不代表具有实际车体安全间隙。

复现修复前结果，在同一环境中对基准提交建立独立工作树：

```bash
git worktree add --detach ../CCPP-before c59ef20dc986fa8df50837ca34361a2dbb4a0b09

PYTHONPATH=../CCPP-before/src python -m fieldplan_offline.cli.plan_boustrophedon \
  --geojson field.geojson --method continuous --spacing 1 --margin 0.2 --step 1 \
  --out out/before_continuous.json --out_csv out/before_continuous.csv
PYTHONPATH=../CCPP-before/src python -m fieldplan_offline.cli.plan_boustrophedon \
  --geojson field.geojson --method bcd --spacing 1 --margin 0.2 --step 1 \
  --out out/before_bcd.json --out_csv out/before_bcd.csv

python scripts/validate_plan_collisions.py \
  --geojson field.geojson --plan out/before_continuous.json --out out/before_continuous.audit.json
python scripts/validate_plan_collisions.py \
  --geojson field.geojson --plan out/before_bcd.json --out out/before_bcd.audit.json
```

两次修复前校验均预期返回 1。不要在未处理预期非零返回值的 `set -e` 脚本中串行运行它们。依赖版本变化可能改变自动扫描角、交点和分段数量，应使用同一环境进行前后比较。

## 几何保证的边界

- A* 的失败表示“当前网格分辨率和端点接入策略未找到安全路径”，不证明连续空间绝对不可达。粗网格会保守地拒绝窄通道；可降低 `--connect_cell` 再试，仍不可回退到危险直线。
- 端点采用最近可见网格点接入；网格离散化和这种接入策略不保证连续空间的完备性或全局最短路。
- 检查针对平面米制坐标及 `kind/type=field/obstacle`，没有新增对 `wall`、`corridor` 等其他 GeoJSON 类型的语义处理；与现有加载器口径一致。
- 规划失败不会删除已有同名输出文件；调用方必须检查进程退出码，不能把旧文件当成新成功结果。
- 不改变历史示例，不让旧问题被新文件掩盖。历史文件应继续被视作不可直接执行的示例。

## 最小转弯半径与 Dubins 的后续接入

以下是下一阶段设计，未在本补丁中实现：

1. 把连接器输入从两个 `(x,y)` 改为两个边界姿态 `(x,y,yaw)`。起始 yaw 来自前一覆盖行行驶方向，终止 yaw 必须匹配下一行方向；后置 `xy_to_poses()` 计算切线方向不能代替航向约束。BCD 单个 `row` 内部的转场也必须处理。
2. 建立车体参数：最小半径 `R_min`、参考点、轮距/轴距、最大转角、实际外轮廓及安全余量。理想自行车模型中 `R_min = L/tan(delta_max)`，限曲率 `|kappa| <= 1/R_min`，运动方向限定为前进。模型参数应来自实际车辆，而非本次任意设置的 margin=0.2。
3. 在行端连接层枚举有效的 Dubins 候选（LSL、RSR、LSR、RSL、RLR、LRL），逐一验证整条曲线及车体扫掠区域，选择可行候选。Dubins 解决无障碍环境中的前进、限曲率连接；最短候选仍可能撞障碍，不能生成后直接接受。[1]
4. 若候选均被挡住，应在 `(x,y,yaw)` 空间采用前进运动基元的 Hybrid A* / 状态格，或用 Dubins 状态空间的采样规划。现有二维 A* 可提供粗略通道；不能对其直角拐点随意倒圆就宣布可行。此处是本项目的建议架构。
5. 对弧线需要连续碰撞验证，或有误差界的保守离散验证。仅检查采样点，甚至仅检查采样点连成的弦，都可能漏掉圆弧外侧碰撞；圆弧弦误差可按 `e=R*(1-cos(delta_theta/2))` 控制，并在碰撞检测中保守计入。使用 OMPL 仍须提供状态和运动有效性检查器。[2]
6. 在整条路径及段间边界验证航向连续、曲率界、车体净空、前进方向；再加入转向速率、速度/加速度与跟踪误差约束。Dubins 曲率在直线/圆弧切换处可突变，若车辆要求转向连续还需曲率连续连接并重新碰撞检查。可能需要预留地头转弯空间、缩短覆盖行或调整行访问顺序。

现有 Plan JSON 已能保存姿态，可保留导出接口；未来可在 `meta` 增加运动模型、R_min、验证级别等信息，但不能给本补丁输出标记“运动学可行”或“已在 Nav2 验证”。

参考一手资料：

[1] Steven M. LaValle, *Planning Algorithms*, §15.3.1 Dubins Curves: https://msl.cs.uiuc.edu/~lavalle/planning/node821.html

[2] OMPL, State Validity Checking: https://ompl.kavrakilab.org/stateValidation.html
