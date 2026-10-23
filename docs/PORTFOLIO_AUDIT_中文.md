# CCPP Mowing Planner v2.0.1：代码复核、修复与作品集证据

日期：2026-10-22。输入是用户上传的 `CCPP_Mowing_Planner_v2.0.0 (1).zip`。本轮只审计该 Python/Tk 项目；没有复核用户另外两个项目的全部源码，也没有部署 GitHub 网站。

## 结论

原项目的常规生成链路通过了原有自动测试和 GUI 检查，但不能据此说“完全没有问题”。本轮通过额外的错误输入/保存结果篡改试验，确认三类结果复验遗漏，并在 v2.0.1 修复。新增研究车辆标签、CLI 自定义车辆输入和网页演示，不改写原覆盖算法为另一套算法。

在本轮有限测试范围内，生成并验证的轨迹没有发现未报告的碰撞、曲率超界、目标遗漏或连续性断裂。此结论不是任意地图、任意参数、任意软件环境下的正确性证明，也不是实车安全认证。

## 1. 实际执行范围

| 检查 | 结果 | 范围与解释 |
|---|---:|---|
| 用户原包 pytest | 170/170 通过 | 重新运行，不是照抄旧报告 |
| 修复版 pytest | 194/194 通过 | 原测试 + 24 项新增回归/参数化用例 |
| 原版真实 Tk GUI | 28/28 通过 | Xvfb 中真实控件、子进程、动画、保存、取消 |
| 修复版真实 Tk GUI | 28/28 通过 | 同一流程重新检查 |
| 原七组示例 | 7/7 重建并复验 | 新运行目录，不直接用原图代替测试 |
| 补充检查 | 50/50 组通过 | 详见下一段；不是 50 个独立随机任务 |
| 大场景 | 2/2 通过 | 默认停转走及 Greedy+Bézier；均完成 1388/1388 目标 |
| 新网页场景 | 8/8 通过 | 新输入、实际 run、save_result 与 validate_saved |
| 网页浏览器检查 | 50/50 通过 | DOM、八组切换、播放/暂停、节点表、四视频、手机/电脑布局 |
| Python 与 JS 一致性 | 670 个时刻 | x、y、航向、速度、角速度最大差 1.52e-14 |

50 组补充检查包括：100 张带非负欧氏下界边权的随机图上 A* 对照 Dijkstra（计为一组）；12 种覆盖策略 × 2 种网格的完整流水线（24 组）；6 种连接算法 × 3 个种子的真实阻挡绕障（18 组）；7 个示例的独立重验与额外采样检查（7 组）。

对其中 6 组曲线示例，额外计算了 29,602 个完整矩形车体位姿及间隙，并用数值差分交叉核对位置导数与航向导数。它是解析连续约束验证之外的补充，不以稀疏采样代替连续检查。RRT 等方法在本轮有限种子通过，不表示有限预算永不失败。

环境：Linux / Python 3.13.5；NumPy 2.3.5、SciPy 1.17.0、Shapely 2.1.2、NetworkX 3.6.1。没有 Windows 或实体机器人执行环境，因此没有将 Linux 结果宣称为 Windows 实测。Python compileall 静态编译通过；Ruff 未预装且当前网络无法安装，因此未将其列为已完成检查。

浏览器说明：容器内受管 Chromium 禁止文件 URL 和回环地址导航。本轮通过 `set_content` 装载页面，内联字节相同的本地 JS/CSS/图片/视频，实际检查 DOM、脚本和视频解码；本地文件路径及 HTTP 资源另作检查。这不是对已上线 GitHub Pages 的远程验收。

## 2. 原版确认的问题及修复

### 问题 A：禁用原地转向的策略未在保存结果复验时重新检查

复现：选含 6 个 TURN 的已生成曲线结果，把配置中的 `curve_allow_pivot_fallback` 改为 false。原验证仍返回 valid=true。

修复：曲线独立验证器明确拒绝与该配置冲突的 TURN；原来对 bounded_curvature 的禁止车体旋转检查继续保留。修改位置 `ccpp_mowing/curve_validation.py`。

### 问题 B：刀具切换等待时间可以被缩短或删除而未被复验拒绝

复现：机器人设定 spinup=2.0s，将 WAIT 缩短为 0.001s 并重排时间线，原验证仍通过。

修复：新增刀具状态转换检查，验证启刀、停刀等待下界、工作前状态、结束关刀；配置确实允许零等待时仍支持。旧版直线轨迹和新版曲线轨迹共同使用该检查。修改 `validation.py`、`curve_validation.py`。

### 问题 C：展示摘要与轨迹本体可能不一致

复现：保存结果的总时长改为 1s、声明面积覆盖改为 100%、事件数改为空，原 `validate_saved()` 仍通过。

修复：重新计算并核对核心数值和事件摘要；非有限值拒绝。严格模式下也拒绝把原必做任务转移到 skipped 记录来绕过目标义务。旧版快照加载改为同样经过独立复验。修改 `pipeline.py`、`snapshot.py`。

这三类问题证明了原复验器存在漏检，**不等于原默认规划器本来就在生成这些被人为改坏的结果**。测试不会伪造一个普通运行碰撞案例来夸大修复。

### 展示与复现实用改进

不同尺寸机器人不再统一标为默认割草机尺寸。GUI 打开自定义车体结果后会更新尺寸文字；命令行新增 `--robot`，可读取平铺 SI 参数或保存的 `robot.json`。新测试覆盖两种输入形式。品牌尺寸只作为原项目历史参考，动态限制均按研究假设解释，不制作品牌性能排名。

## 3. 新制作的八组结果

所有长度为实际曲线轨迹长度；模拟时间包含停转、刀具等待等事件。面积覆盖列使用声明作业区分母，原始田块分母另在网页及 metrics.json 中报告。

| 场景 ID | 已服务/候选目标 | 计划访问项数（含回访） | 长度/m | 时间/s | 声明区覆盖 | 原地回退 | 被阻挡的 A* 查询记录 |
|---|---:|---:|---:|---:|---:|---:|---:|
| compact_greedy_astar | 131/131 | 196 | 108.00 | 2085.4 | 96.82% | 18 | 0 |
| mower_mst_astar | 113/113 | 159 | 93.39 | 1264.2 | 93.50% | 28 | 0 |
| compact_mst_astar | 131/131 | 185 | 105.07 | 2029.7 | 98.21% | 31 | 0 |
| obstacle_astar_bezier | 113/113 | 114 | 111.57 | 835.2 | 96.19% | 2 | 8 |
| compact_ga_astar | 131/131 | 132 | 87.16 | 1478.7 | 96.33% | 18 | 0 |
| compact_greedy_conservative | 131/131 | 196 | 108.00 | 2814.2 | 96.82% | 18 | 0 |
| forward_dubins_r060 | 214/214 | 215 | 171.46 | 913.5 | 99.91% | 0 | 0 |
| forward_dubins_r090 | 214/214 | 215 | 240.07 | 981.2 | 99.91% | 0 | 0 |

`compact_greedy_astar`、`compact_mst_astar`、`compact_ga_astar` 固定同一地图、紧凑车辆、网格、曲线和时间方法，只对比覆盖顺序；这是单图单种子的对照，不能宣称 GA 总优于 MST 或贪心。

`compact_greedy_conservative` 与优化版使用相同曲线几何，仅改变时间参数化。保守方案 2814.2s、SLSQP 2085.4s，约减少 25.90%。这是固定路径下模拟时间的改善，不是实测节能率或全局最优证明。

前进模型的两组使用相同任务顺序和车体，仅将最小半径从 0.60m 改为 0.90m。均无 TURN；局部连接形状及总长度变化明显。曲线可存在回环或自交，但必须满足障碍与运动约束，自交本身不等于发生碰撞。局部合法解不必是整个任务最短路线。

## 4. 节点顺序与 A* 的证据

网页节点编号 1、2、3…对应 `ordering.order` 中的计划访问项。每项保留任务节点 ID、x/y、该节点第几次出现；同一节点可回访。网页可从任意访问序号开始在图上标注 24 项，完整表与 CSV 不截断。

三种顺序要区分：覆盖任务的计划访问顺序；A* 搜索弹出的状态顺序；A* 最终返回路径的节点顺序。它们也都不等于割刀首次覆盖各点的时间顺序。曲线按 cutter_footprint 完成服务时，车体中心可能绕开原折角。

真实绕障示例来自 `obstacle_astar_bezier` 中一条被障碍挡住的已记录查询：起点 (6.9,3.9)，终点 (2.7,3.9)。直连不安全；A* 展开 85 个状态，Dijkstra 展开 202 个状态，两者图路径代价均为 8.115265479m。完整路径含 16 个导航节点；-1/-2 为挂接起终点。

图中展示搜索得到的图路径，JSON 同时保存后续可见性捷径后的实际连接。安全捷径后路径可短于原图路径；这不会改变“原有限图查询代价与 Dijkstra 相同”的结论。不能把该结论推广为覆盖路线、曲线轨迹或时间全局最优。

网页同时报告实际 A* 非缓存次数。紧凑车对比案例关闭 direct_visible，因此即使两点可直连也会走 A*；不把这些查询都说成“被障碍迫使绕行”。两组前进 Dubins 演示实际都是 verified_visibility 连接，页面明确显示 A* 搜索次数为零。专门障碍案例才有 8 条被阻挡的 A* 记录。

## 5. 运动模型与未实现能力

差速模型允许明确记录的原地转向回退；前进曲率受限模型禁止车体原地转向，但允许静止转舵。验证包含速度、加速度、jerk、航向导数、横向加速度、最小移动转弯半径和转舵速率等。左右履带量对前进研究模型仅为等效驱动包络，不能直接作为真实 Ackermann 车轮命令。

每个基本运动段起止静止。因此图形曲线连续，不代表整项任务永不停车；半径变大、约束变严或频繁切换会增加时间。截图对应的大型 Greedy 任务仍有 1435 次原地回退，局部短折返不能一律改成大半径曲线。该组完成 1388 目标但原始区面积覆盖约 73.75%、声明区约 98.45%，仍需人工收尾。

没有闭环跟踪控制器、定位噪声、实际执行器、轮胎/履带地面接触、质量/力矩模型；没有 Hybrid A* 或 Reeds–Shepp。网页和视频是同一解析参考轨迹的回放，不是“仿真机器人已经证明能跟踪”或“实车已执行”。几何与连续运动约束在模型内复验，通过不代表工业功能安全认证。

局部曲线转换可能因为窄通道、半径、目标服务约束而明确失败。参数修改失败应先阅读 error.log 和终端原因，不应删除碰撞检查来强行出图。

## 6. 可复现运行

在 v2.0.1 根目录，选择正确虚拟环境解释器。

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements-dev.txt
.\.venv\Scripts\python.exe -m pytest -q
.\.venv\Scripts\python.exe tools/audit_extended.py --out outputs/audit_new
.\.venv\Scripts\python.exe tools/build_portfolio_cases.py --out outputs/portfolio_new
```

`build_portfolio_cases.py` 默认重建八组结果，拒绝覆盖已有目标目录。仅重建 A* 障碍示例：

```powershell
.\.venv\Scripts\python.exe tools/build_portfolio_cases.py --out outputs/astar_new --ids obstacle_astar_bezier
```

GUI 使用 `Open saved result...` 打开 `portfolio_examples/<场景ID>` 目录。命令行重新生成某组时，需同时使用它的地图、配置与机器人参数，不能只换 config：

```powershell
.\.venv\Scripts\python.exe -m ccpp_mowing plan --config portfolio_examples/compact_greedy_astar/config.json --map portfolio_examples/compact_greedy_astar/map.geojson --robot portfolio_examples/compact_greedy_astar/robot.json --out outputs/compact_rebuilt
.\.venv\Scripts\python.exe -m ccpp_mowing validate outputs/compact_rebuilt
```

网页素材生成器 `tools/export_portfolio_assets.py` 依赖项目现有库；输出 MP4 额外需要系统 FFmpeg。GUI、曲线求解、网页浏览均不依赖 FFmpeg。只重建图与数据可加 `--skip-video`，会保留已有视频清单。

```powershell
.\.venv\Scripts\python.exe tools/export_portfolio_assets.py --runs outputs/portfolio_new --site D:/robot_portfolio
```

## 7. 证据文件与版本管理

修复版的 `docs/audit_2026-10-22/` 包含原包复现证据、pytest 日志与 XML、补充测试摘要、GUI 检查清单、网页检查摘要和大场景摘要。`portfolio_examples/` 保存完整八组输入与结果。`MANIFEST.sha256.json` 检查交付文件；网页的 `media_manifest.json` 记录事件来源哈希和视频时刻/帧率。

本包保留原 v2.0.0 的 docs/qa、测试报告与示例作为历史材料；新结论以本报告及 `audit_2026-10-22` 为准，不把旧截图日期当成本次运行日期。项目许可证与第三方来源说明保留，未替用户重授版权。
