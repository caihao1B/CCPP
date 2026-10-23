# CCPP Mowing Planner：最终研究版使用说明

本项目是**学生小论文背景下的农业网格全覆盖实验工程**。主线是正方形／等边三角形网格、连通分簇、簇间排序、簇内覆盖、A*／RRT／PRM 连接、差速有限速率运动以及可核查的人工残留。纯 Python，不需要 ROS、Gazebo 或毕业论文执行系统。

## 1. 第一次启动

请解压到**新的目录**，例如 `D:\python_projects\CCPP_Mowing_Planner`。不要覆盖以前的 `CCPP_Grid_Python`，也不要复制旧 `.venv` 或旧配置。

在 VS Code 用 Open Folder 打开整个目录，进入 Terminal。最稳妥的方式是不依赖激活脚本：

```powershell
py -3.13 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe app.py
```

然后执行 `Python: Select Interpreter`，选择项目自己的 `.venv\Scripts\python.exe`。以后可以直接点击 Run Python File；Run Test 是测试，不是启动 GUI。Windows 也提供 `install_windows.bat` 与 `run_gui.bat`。

本次实际测试是 Linux/Python 3.13.5；Windows 安装脚本和 CI 配置已提供，**没有声称 Windows 实机已经通过**。GUI 使用 Tk：Windows 官方 Python 安装时保留 Tcl/Tk 选项。Linux 若提示缺少 `_tkinter`，需要为对应系统 Python 安装 tkinter 支持；这不是 pip 里的第三方 `tkinter` 包。

## 2. GUI 怎么用

运行 `app.py` 后，左边选项可以滚动；底部 Generate path 和 Cancel 按钮保持可见。

- Map source：generated 生成图、original 原始农业图、imported 导入本地米制 GeoJSON。
- Field shape / Obstacle density / Distribution：形状、障碍密度和分布。导入／原图模式不重新生成障碍，相关生成参数不改变已有地图。
- Grid：square / triangle。默认 equal_area、单元 0.12 m²，不能把 equal_side 误当作相同分辨率。
- Partition：bfs、kmeans 或 none；Max cells/cluster 表示每簇节点上限，不是目标簇数量。
- Intra-cluster / Inter-cluster / Connector：分别选择任务访问顺序、簇间顺序和具体连接算法。
- Time profile：slsqp 是实际数值时间优化；conservative 是已知可行基线。
- Failed query：astar_reported 允许**有记录的 A* 补救**；strict 不允许把失败算法改名为成功。

点击 Generate path 后，规划在独立子进程中执行，界面可取消。成功才加载结果。Path & animation 页支持播放／暂停、倍速、时间轴、First turn 慢速跟随和导出图片；Speed 页显示计划速度；Metrics & audit log 页说明真实调用方法、残留及回退。

动画来自同一条分析运动轨迹，不是实车录像。First turn 特别适合观察完整车体、左右侧速度和连续朝向。停止动画不会停止已经完成的路径数据；Cancel 用于尚在计算的任务。

## 3. 不使用 GUI 也可以完整运行

```powershell
.\.venv\Scripts\python.exe main.py
.\.venv\Scripts\python.exe main.py --grid triangle --intra mst_euler --inter ga --planner prm --out outputs\triangle_mst
.\.venv\Scripts\python.exe main.py --map maps\original_field.geojson --config configs\original_field.json --out outputs\original
.\.venv\Scripts\python.exe validate_saved.py outputs\original
```

无参数 `main.py` 使用默认值并覆盖 `outputs/latest`。指定目录时默认拒绝覆盖已有结果；确需覆盖时使用 `--overwrite`。复现实验请新建不同输出目录。

旧项目的 `--solver` 在本版改为明确的 `--intra`，簇间使用 `--inter`；不要混用旧命令和配置。`python -m ccpp_mowing --help`、`python -m ccpp_mowing plan --help` 查看完整说明。

生成地图：

```powershell
.\.venv\Scripts\python.exe generate_map.py --shape l_shape --width 24 --height 18 --density high --distribution clustered --seed 9 --out maps\my_field.geojson
```

坐标单位为米。经纬度数据必须先独立投影到本地米制坐标，不能直接拿 50°／10° 当作几十米。

## 4. 采用的机器人与放弃策略

车体 1.10×1.05 m、刀幅 0.70 m 参考 Agria 9500-70 厂家数据。产品本身是遥控履带割草机，本项目是它的**尺寸参考模型**，不是经过实车标定的数字孪生。等效驱动间距、加速度、jerk、工具等待等是研究假设，详见 [ROBOT_MODEL](docs/ROBOT_MODEL.md)。

采用保守但清晰的停—转—走：前进直行、停车、有限速率原地转向、继续前进。不使用整车倒车，不以几何曲线小环伪装可执行转弯。工具关闭时不记作业面积；刀片停转时间没有厂家认证，仿真等待不能当作实际安全规程。

默认预先声明边界与障碍附近 0.80 m 带为人工补边范围，但保留原图作为总面积分母。过窄空间、无安全旋转姿态、不可达组件、随机搜索超时分别记录。已完成区域仍可通行。严格模式可以要求失败即停止；普通演示模式可以保存“已经验证的部分任务＋明确人工残留”。

**全体中心访问 100% 不等于割草面积 100%。**工程独立保存原图面积覆盖率、声明区覆盖率、人工残留以及必访中心完成率。三者不要互相替代。

## 5. 数值优化到底做了什么

本版是**固定直线／旋转运动原语的受约束时间曲线优化**，不是 CHOMP、TrajOpt、MPC，也不是全局非线性车辆轨迹优化。SLSQP 优化每段峰值速度与加速持续时间，在速度、加速度、jerk 和左右驱动限值下减少运动时间；候选再次验证失败则回到保守可行曲线。

它不改变必访任务，不把轨迹平滑成小环。时间改善比的是同一条运动路线的保守时间曲线，不是商用设备效率或上一个项目版本。高密度中心任务和频繁工具开关可能造成很长的模拟工时，这是该基线的局限，不能包装成现实生产能力。

## 6. 关键结果文件

| 文件 | 内容 |
|---|---|
| `coverage.png` | 网格、分簇、作业／转场及最终人工区 |
| `speed.png`, `turn_tracks.png` | 全程计划速度、一个转向的左右驱动 |
| `metrics.json` | 面积、中心、距离、时长、实际算法与回退 |
| `validation.json` | 独立几何与运动检查，不依赖求解器 success |
| `ordering.json` | 各簇、入口出口、顺序、算法替代原因 |
| `queries.json` | 请求算法、真实使用算法、可见直线及补救 |
| `events.json`, `trajectory.csv` | 分析事件和按时间采样；动画使用同一事件 |
| `manual_handover.csv`, `coverage.geojson` | 原图未完成区域、声明区额外残留及原因 |
| `grid.geojson`, `skipped_tasks.json` | 候选中心、被排除格子与跳过任务 |
| `status.json` | 最后写入的完成／失败／取消标志 |

有剩余区域的有效结果标记 `verified_with_manual_residual`，不是失败，也不是隐瞒残留的“全部完成”。当任务没有任何有效可行中心或不能安全返航时，程序明确报错，不画一条假完成路线。

## 7. 测试和规模

[测试报告](docs/VALIDATION_REPORT.md) 提供本次实测证据。`tests/` 是自动化测试；`tools/release_matrix.py` 是算法组合与地图组合；`tools/dense_integration.py` 才是较密任务测试。984 组并不意味着所有地图×所有算法×所有种子全部穷举。

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements-dev.txt
.\.venv\Scripts\python.exe -m pytest -q
```

原图默认约 5300 个任务中心。0.24 公顷细网格实测约 1.63 万任务。1 公顷例子是**粗网格规模检查**，不能展示成高覆盖农业成功案例。默认 30000 格、180 个局部代价节点等预算用于防止桌面内存失控；超限说明原因，不默默降低分辨率。

## 8. GitHub 与求职展示

直接展示英文 README、GUI 图、转向 GIF、双网格例子、配置与实际测试报告。CI 已配置，但首次上传后需要等 GitHub Actions 实际运行，不预先声称全部平台通过。

保留 `docs/PROVENANCE.md` 与 `NOTICE.md`。原库未声明许可证，因此本次未替你擅自赋予 MIT 等许可；决定公开复用范围后再添加合适 LICENSE。不提交 `.venv`、缓存、私钥、旧运行目录或大视频。

具体面试表述和代码阅读顺序见 [PORTFOLIO](docs/PORTFOLIO.md) 与 [ALGORITHMS](docs/ALGORITHMS.md)。强调可验证的模型、算法与实验，不声称实车落地、标定精度或硬实时控制。
