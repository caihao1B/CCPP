# v2.1.0 更新入口

请先读 [START_HERE_中文.md](START_HERE_中文.md)。新展示结果在 `comparison_examples/`，推荐08—12预设；原01—07和下面的旧说明保留为兼容基线。新策略、复验、网格/算法对照详见 [本版报告](docs/V2_1_测试与展示报告.md)。

---

> **v2.0.1 审计与作品集补丁**：本轮发现及修复、194项测试与八组展示结果见 [审计报告](docs/PORTFOLIO_AUDIT_中文.md)，演示入口见 [作品集指南](docs/PORTFOLIO_中文.md)。以下原 v2.0.0 功能说明继续适用；历史测试数字不代表本轮统计。

# CCPP Mowing Planner v2.0.0：可验证曲线轨迹与机器人动画

基于本次上传的 `CCPP_VSCode_Interview_Kit.zip` 中 GUI2 改造。此包是 **GUI2 的独立升级版**，不替换 GUI1。运行步骤先看 [START_HERE_中文.md](START_HERE_中文.md)。所有图片/GIF 均由本项目实际地图、曲线和事件生成，不是示意图片。

## 交付功能

保留已有方格/三角格、BFS/K-means 分簇、贪心、MST–Euler、TSP/GA、蛇形扫描、六种连接规划器及其报告。新增：

- 机器人 `differential` / `bounded_curvature` 两种模式。
- `stop_turn_go`、`circular_fillet`、`bezier_g2`、`bezier_slsqp`、`dubins` 五种方案。
- 独立的时间参数化选择：`slsqp` / `conservative`。几何优化与时间优化不是同一个开关。
- GUI 参数包括最小移动转弯半径、局部倒角长度、作业/转场速度、加速度、jerk、横向加速度、等效轴距、转舵速度、轨迹空间采样步长、服务标准及差速回退权限。
- 全车体曲线碰撞包络、连续导数界、独立保存结果重验。
- 车体沿解析曲线运动、航向切向对齐、割刀与已走轨迹显示、曲率图、GIF 导出。
- 7 个可应用的 GUI 预设、已计算的示例结果、验证记录及本地开发说明。

## 快速运行

Windows：`setup_windows.bat` → `run_gui.bat`。每个新项目独立创建 `.venv`。运行脚本不会安装 Codex，不需要 OpenAI API key，不连接真实机器人。

Linux/macOS：在有 Python >=3.11、Tcl/Tk 和图形显示环境的电脑上执行 `bash setup.sh`，再 `bash run_gui.sh`。Linux 缺少 Tk 或 venv 时须通过相应系统包管理器补齐。无显示环境可使用 CLI；GUI 自动测试在 Linux 使用 Xvfb。

在项目根目录、正确解释器下：

```powershell
# 启动 GUI
.\.venv\Scripts\python.exe app.py

# 首选曲线示例：优化 Bézier
.\.venv\Scripts\python.exe -m ccpp_mowing plan --config configs/demo_03.json --map maps/curves_open.geojson --out outputs/my_bezier

# 前进 + R_min=0.60 m 的 Dubins 示例
.\.venv\Scripts\python.exe -m ccpp_mowing plan --config configs/demo_04.json --map maps/forward_open.geojson --out outputs/my_dubins

# 从磁盘重新读取并独立验证
.\.venv\Scripts\python.exe -m ccpp_mowing validate outputs/my_dubins

# 从同一个事件模型导出弯道动画
.\.venv\Scripts\python.exe -m ccpp_mowing render outputs/my_dubins --out outputs/my_dubins/first_curve.gif
```

输出目录非空会被拒绝，重复实验请换目录；确实要替换时显式加 `--overwrite`。`main.py` 仍是仅规划的兼容入口，完整子命令使用 `python -m ccpp_mowing`。`Config()` / `configs/default.json` 保持原停转走默认值，避免破坏旧脚本；**GUI 初始曲线选项与演示预设选择优化 Bézier**，并不是所有无参数 CLI 都自动改成曲线。

## GUI 参数怎样理解

左侧 `Map / coverage` 是旧的任务和路径参数，`Motion / curves` 是新增运动参数，两页都能滚动。右侧有 Path & animation、Speed、Metrics & audit log、Curvature & constraints。

| 参数 | 实际控制的内容 |
|---|---|
| Motion model | 能否在零平移时改变车体航向；bounded_curvature 禁止 TURN |
| Trajectory method | 几何曲线的构造或局部数值优化方法 |
| Moving R_min | **运动曲线**的最小半径；差速原地事件是单独模式，不由有限曲率表达 |
| Time scaling | 固定曲线的时间优化或保守速度剖面 |
| Max blend trim | 局部允许使用多少原直线长度用于圆滑转弯，不是任意改写覆盖顺序 |
| Work/transit speed | 与继承 Robot 上限取较严格者；不会超过原模型硬上限 |
| Acceleration / jerk | 沿路径标量速度的一、二阶时间导数限制 |
| Lateral acceleration | 限制转弯时的 `abs(kappa) * v**2` |
| Wheelbase / steer rate | 前进模式的等效自行车转舵关系与转舵速度约束 |
| Spatial sample step | 空间 CSV/图像线性近似的密度；GUI 位姿来自解析式，不由此步长跳点 |
| Service rule | cutter_footprint：目标被开刀扫过；centre_exact：保守要求原中心严格服务 |
| Pivot fallback | 仅差速有效，不能混淆为连接算法 A* 的回退策略 |
| Manual setback | 明确的边界/障碍周围预声明收尾带；扩大它会改变任务分母，必须披露 |

半径必须为正。差速可以原地转，但移动圆弧仍用正半径；不通过输入 R=0 来启用原地转向。前进模式初始航向选择第一条轨迹切向，不会在任务开始偷偷原地转向。

## 这不是“只把图画圆”

`events.json` 中每个移动基本段存完整解析几何（直线、圆弧或 6 个控制点）和时间剖面。GUI、GIF、CSV 导出共用 `motion.evaluate_events()` / `curve_motion.evaluate_curve_event()`。其速度方向与车体朝向满足同一条理想非完整运动关系；曲率来自几何导数，角速度由 `omega=kappa*v` 得到。加载保存结果也会重算验证，不信任旧的成功标签。

## 输出文件

| 文件 | 用途 |
|---|---|
| `raw_legs.json` | 原始折线任务段，曲线前后比较 |
| `events.json` | 唯一权威解析轨迹与时间事件，含 TURN/STEER/WAIT 等 |
| `trajectory_reference.csv` | 带时间的 20 列参考数据，包括 x/y/yaw/v/omega/加速度/等效左右速度及曲率、转舵、横向加速度、jerk |
| `curve_path.csv` | 7 列空间采样：x,y,yaw,kappa,event_id,tool_on,u；不含时间，不能代替时间轨迹 |
| `trajectory.csv` | 保持旧 14 列格式的时间采样 |
| `curve_diagnostics.csv` | 曲率、等效转角与动态诊断，含时间及事件编号 |
| `optimization.json` | 几何/时间求解记录、实际方法、保守回退、优化前后目标 |
| `validation.json` | 全车体、连续导数界、任务与残留的独立复核 |
| `metrics.json` | 实际曲线计数、pivot 数、任务完成、两种面积覆盖率 |
| `coverage.png` / `curvature.png` / `lateral_acceleration.png` | 真正执行轨迹及约束图 |
| `manual_handover.csv` / `coverage.geojson` | 显式人工收尾区域，不隐藏漏割 |

使用 `--no-samples` 时不生成时间 CSV。`--no-plots` 不影响规划与验证。导出 CSV 有数值采样误差，权威模型是解析事件，而非用离散采样间隔证明安全。

## 安全与结果解释

曲线段用车体矩形的外接圆作保守全方向包络；直线段用完整矩形扫掠。这个检查可能拒绝真实可通行的狭窄曲线，不应被解释成完整可达性证明。无实测打滑、定位漂移、轮胎侧偏、电机或割刀负载模型。尺寸继承旧包，转舵/履带/速度参数含研究假设，不是 Agria 官方车辆动力学模型。

所有基本曲线段在边界静止。Bézier 局部补片与直线可 G2 相接，但混合了 Dubins 圆弧的整体路线未必处处曲率连续。圆弧曲率跳变处在前进模式增加有限时间 STEER，车体不转向；这样是保守、可解释的参考运动，不是全局时间最优或不停顿高速运动。

**前进模式不能对任意密集锯齿折线承诺成功。** 当前是固定任务顺序的局部曲线转换器，不是 Hybrid A*、Reeds–Shepp 或全局运动学覆盖重规划。失败时保留失败状态；不自动降低半径、不穿障碍、不跳删目标、不回退原地旋转。

曲线模式目标完成与面积完成分别核算。圆滑拐角可以不经过原车体中心，但必须满足所选的原始目标服务条件。预设05是粗网格运动学演示；全部目标被扫过并不代表全部面积被扫过。

## 测试与后续开发

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements-dev.txt
.\.venv\Scripts\python.exe -m pytest -q
.\.venv\Scripts\python.exe tools/check_manifest.py
.\.venv\Scripts\python.exe tools/build_curve_demos.py --ids 03 04 --out outputs/rebuilt --gifs
```

测试证据见 `docs/VALIDATION_REPORT.md` 与 `docs/qa/`。本轮已实测 Linux Python 3.13.5；Windows BAT/VS Code 配置已编写检查，但未在你的 Windows 系统实际执行。Ruff 未成功安装，未计入已通过检查。

源码详细说明：[轨迹实现](docs/TRAJECTORY_DESIGN_中文.md)，[演示顺序](docs/DEMO_GUIDE_中文.md)，[Codex后续任务](docs/CODEX_NEXT_STEPS_中文.md)。旧版文档位于 `docs/legacy_v1`，不是 v2 测试证据。旧 `tools/release_matrix.py` 等是历史批量工具，日常运行不需要执行它们。
