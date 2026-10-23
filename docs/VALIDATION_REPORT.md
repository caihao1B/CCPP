# v2 发布验证记录

## 实际执行结果

| 检查 | 结果 | 证据 |
|---|---:|---|
| 原GUI2自动测试 | 110通过 | qa/pytest_legacy.txt / xml |
| 当前完整测试（包含原测试） | 170/170通过 | qa/pytest_all.txt / xml |
| 新增曲线测试 | 60项，包含在上行，不重复相加 | tests/test_curved_motion.py |
| 实际Tk GUI操作 | 28/28通过 | qa/gui_curves.json |
| 已保存预设独立重验 | 7/7通过 | qa/demo_verification.json |
| 解析曲线GIF | 03/04/05已生成 | examples/demo_XX/first_curve.gif |
| 稠密场景 | serpentine和greedy均完成1388/1388原目标 | qa/dense_serpentine.json, dense_greedy.json |
| 曲线上“边平移边转向” | 03/04/05各41个测试时刻同时v>0且omega非零 | qa/moving_curve_evidence.json |

GUI检查实际调用了Generate按钮、真实规划子进程、动画时钟、曲线求值、前进模式禁TURN、图片/GIF保存、坏半径拒绝、取消与子进程回收，不只是控件存在性检查。图像来源是实际Tk窗口。

测试包含：随机Dubins端点/半径，G2端点，几何保守曲率界，弧线中间障碍检测，八组方法/模型组合运动导数与有限差分一致性，无法转弯必须失败，取消，以及篡改控制点、半径、时间、目标清单、刀具状态、指标、转舵跳变后拒绝。独立重验会重新生成原任务清单，不只检查文件哈希。

## 环境和未执行项

实际环境：Python 3.13.5 / Linux。科学库版本见requirements-tested.txt和VALIDATION_REPORT.json。真实GUI测试使用Xvfb，不代表Windows操作系统或物理屏幕实测。Windows安装/运行脚本和VS Code配置已提供，但未在用户Windows机器执行。

Ruff未安装，联网安装尝试因当前运行环境DNS失败而未完成，因此**Ruff未执行**，不计入通过项。compileall为独立语法检查。CI配置用于未来执行，不是当前已通过的云端证据。

本轮没有实车、Gazebo、ROS2、闭环Pure Pursuit/MPC或定位误差试验。限幅数据中包含研究参数，不能当厂商标定。

## 不能从验证得出的结论

- 不等于所有地图、任意半径都可规划，前进模式局部转换会正确失败。
- 不等于整个田块面积100%覆盖，预设05粗网格只有66.85%声明区覆盖。
- 不等于不停车或全局最短时间，基本段为起止静止。
- 控制凸包/曲率/扫掠是保守浮点界和几何计算，不是形式化证明器或物理安全认证。
- 软件回放中的参考车体完全在路径上，不意味着有跟踪误差的实车也能做到。

## 来源与变更

上传ZIP的SHA256：`23415bb9687cae5f6c9117c4aa614b08f75d1122e61dfb18375c99ea35464074`。

原模块逐文件对照见SOURCE_CHANGES.json。原地图生成、网格、几何、覆盖排序/分簇、搜索、导航模块保持原字节；新增与接入变化集中于曲线/时间/验证/GUI/CLI。旧文档已归档legacy_v1，旧QA不作为本次新功能证据。

代码后续修改会使发布manifest失配，这是正常的版本差异提示；只有主动重建并重新验证后才能更新发布哈希。
