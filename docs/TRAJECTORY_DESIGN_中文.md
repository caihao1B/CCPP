# v2 轨迹实现：从代码讲到动画

本说明对应当前 ZIP 代码，不把未来的 Hybrid A*、Reeds–Shepp、MPC 或真实闭环跟踪记作已实现。

## 一、真实调用链

GUI `App.start()` 组织配置，通过当前解释器启动 `python -m ccpp_mowing plan`。CLI → `pipeline.run()` → 原地图/网格/导航/分簇/排序流程 → 原始 `legs`。

`trajectory_method == stop_turn_go` 时走旧 `motion.build_events`。其他方法依次进入：

```text
curve_motion.build_curve_events
    → curve_planner.build_geometry
        → 合并语义一致的共线直线
        → 局部切线圆弧 / Bézier控制柄 / SLSQP候选
        → 跨行Dubins连接；前进模式必要时尝试Dubins绕行
        → 曲率上界 + 全车体碰撞包络 + 原作业段服务保留检查
    → CurveTimeOptimizer.solve
        → 时间/升速比例约束优化
        → 连续导数界验证，必要时保守重定时
    → WORK / TRANSIT / TURN / STEER / WAIT / SERVICE 事件
curve_validation.validate_curves
    → 从曲线描述与时间剖面重新计算验证，不信任求解器标签
pipeline.save_result
    → JSON / CSV / 图像
snapshot.load_view
    → 新曲线结果独立复验 → GUI或GIF
motion.evaluate_events
    → curve_motion.evaluate_curve_event → 同一时刻的位姿和导数
```

GUI 和 GIF 都是“按时间求值”，没有新增一条只供美化显示的替代路线。测试还用数值差分检查解析位置导数是否等于 `v*(cos(yaw),sin(yaw))`，以及航向导数是否等于 omega。

## 二、新增文件职责

| 文件 | 核心内容 |
|---|---|
| `curves.py` | LINE / ARC / BEZIER5 描述、0–4阶几何导数、曲率、弧长、de Casteljau细分、控制凸包导数界、全曲线碰撞包络 |
| `dubins.py` | 六族Dubins曲线候选；端点、方向和长度检查 |
| `curve_planner.py` | 旧任务折线转曲线、局部SLSQP形状优化、服务保留、正半径硬约束、显式回退 |
| `curve_motion.py` | 时间优化、连续动态约束、事件装配、解析位姿与诊断导出 |
| `curve_validation.py` | 从保存数据独立复核、原目标服务、面积残留与约束统计 |
| `animation_export.py` | 解析事件GIF回放，与GUI共用相同求值函数 |

修改文件主要是 models/pipeline/motion/validation/snapshot/visualization/gui/cli 的接入，不改动原 order_core/ordering/search/navigation 等覆盖和连接算法。`docs/SOURCE_BASE.json` 记录原包源代码哈希。

这里主要是函数和对象组合，而不是复杂继承体系。新增 `CurveTimeOptimizer` 为普通类，调用 SciPy，不继承 SciPy 优化器。曲线和事件是可序列化字典，便于重算与跨进程传递。

## 三、两种机器人模式

### differential

理想差速参考模型：

```text
x_dot = v cos(yaw)
y_dot = v sin(yaw)
yaw_dot = omega
u_L = v - b omega/2
u_R = v + b omega/2
```

移动时由曲线给出 `omega=kappa*v`。如果曲线无法在车体空间、半径和覆盖要求下构造，且允许回退，可以增加有限时间 TURN；它会计入 `pivot_fallbacks`。关闭回退后则拒绝该结果。

### bounded_curvature

参考点沿前向曲线移动：`v>=0`，`abs(kappa)<=1/R_min`，**不允许 TURN**。等效自行车关系 `delta=atan(wheelbase*kappa)`，并限制转舵速度。这个模型不是原 Agria 的 Ackermann 改装复刻；是使用同一矩形尺寸、不同参考运动学约束的研究比较。

参考点的矩形车体中心、等效轴距和转向指示是明确建模约定；不宣称真实前后轴偏置、左右Ackermann角、轮胎侧偏或实际轮速已建模。L/R 数据在此前进模式下是保守的等效差速包络，不是实际Ackermann两前轮电机命令。

初始航向选择第一段切向；回到起点约束位置，不约束最终航向和转舵角。改变这些边界条件必须同时改规划、事件和验证。

## 四、几何方法

### 1. stop_turn_go

保留旧版折线与有限速原地转向，作为对照。只能用于 differential。新曲率/速度参数不应被误认为替换了这一旧分支的全部 Robot 参数。

### 2. circular_fillet

设拐角方向变化为 theta，切点距顶点为 d，则 `R=d/tan(abs(theta)/2)`。默认从要求的最小半径确定切点，并受两侧可用边长与 trim 上限约束。生成实际圆心、起始角、转角描述。

圆弧内部曲率为常数 ±1/R，与直线相接是切向连续，但曲率从0跳到±1/R。时间模型在基本段边界静止；前进模式用 STEER 显式改变转舵角，不假设无穷快执行器。

### 3. bezier_g2

使用五次Bézier：

`r(u)=sum_{i=0..5} B_i^5(u) P_i, 0<=u<=1`。

设入切点A、出切点B、单位切向t0/t1，则控制点为：

```text
P0=A
P1=A+h1*d*t0
P2=A+2*h1*d*t0
P3=B-2*h2*d*t1
P4=B-h2*d*t1
P5=B
```

因此端点二阶参数导数为0，曲率为0，在非零切向条件下可与直线G2连接。固定柄长比例约0.30，尝试可用trim并检查曲率界。Bézier不是天然满足最小半径，也不是任意平滑就不会撞障碍。

### 4. bezier_slsqp

实际优化变量是两个无量纲控制柄 `h1,h2`，在[0.12,0.42]内。局部目标为无量纲弧长加权曲率变化能量，作用是减少曲线绕行与曲率变化；不是电池电耗公式。

候选生成在81个参数位置计算曲率约束和非零切向约束；**这是优化阶段，不是最终安全检查**。优化结果必须再通过连续控制凸包导数界、全曲线车体包络和工具服务保留检查。数值优化失败会记录失败并尝试受约束的基线几何；不会把 `success=True` 当验证结论。

SLSQP求解日志在 `optimization.json`，包括初始/最终目标、迭代和实际接受条件。局部优化的目标降低不代表整条覆盖任务全局最短或任务时间一定最小。

### 5. dubins

枚举 LSL、RSR、LSR、RSL、RLR、LRL 六族，仅前进，圆弧半径不低于R_min。端点是(x,y,yaw)，不只是两点坐标。零位移而方向变化时需要实际绕行，不能替换成原地旋转。

实现先产生几何候选，按长度排序，在R_min及更大半径候选里找通过碰撞包络的连接。若全部失败，拒绝该局部转换。不是在带障碍世界上证明最短的完整Dubins规划器。

圆弧/Bézier方案遇到跨行调头也可调用Dubins，因此是可审计混合方案。整体曲率连续性必须看实际原语，不能因下拉框写着Bezier就宣称全场C2/G2。

## 五、真正的时间参数化

每个几何原语用参数u而不是直接当匀速距离。时间剖面使u从0积分至1，起止速度、加速度为0。设总时间T、升速比例lambda，参数峰速为 `p=1/((1-lambda)*T)`，升速时间lambda*T，匀速时间(1-2lambda)*T。

升速段参数速度是五次函数 `p*(10z^3-15z^4+6z^5)`。它的解析加速度/jerk极值用于约束，不用稀疏CSV采样推断峰值。

几何导数给出：

```text
s(u) = norm(r'(u))
v = s(u) * u_dot
omega = yaw'(u) * u_dot = kappa * v
a = s'(u)*u_dot^2 + s(u)*u_ddot
j = s''(u)*u_dot^3 + 3*s'(u)*u_dot*u_ddot + s(u)*u_dddot
alpha = yaw''(u)*u_dot^2 + yaw'(u)*u_ddot
```

同样推导yaw jerk和等效左右轮的速度/加速度/jerk包络。横向加速度界为 `abs(kappa)*v^2`；等效转舵速度满足 `abs(delta_dot) <= wheelbase*abs(kappa_u)*u_dot` 的保守界。

`conservative`使用可行时间再保留25%余量。`slsqp`实际优化T与lambda，检查所有连续界，增加0.5%时间余量后再验；没有通过则使用已验证保守时间。指标中的time reduction是**相同几何/事件任务的保守时间基线**，不是“比原版总体快了多少”。

段间静止是当前明确限制，即使几何G2也不直接跨段保持非零速度。后续要变成连续通过速度，需要联立邻段边界速度以及新的导数、转舵连续验证，不能只把动画插值改平滑。

## 六、碰撞、曲率与覆盖怎么验

直线：完整矩形平移扫掠；圆弧：精确弧高+弦线包络，再增加车体外接圆、间隙和几何余量；Bézier：de Casteljau细分控制凸包，验证整段包络包含于自由空间。孔洞/障碍边界同样参与检查。

Bézier连续曲率界：在细分区间中，导数曲线位于导数控制点凸包；以凸包到原点距离给切向模长下界，以最大交叉积/范数界给曲率和更高导数上界。无法认证非零切向时拒绝，不生成尖点。

工具覆盖：对WORK曲线有误差界的折线近似作保守内扫掠，减去几何误差余量。原始required任务表由地图/config重新生成，必须逐点满足服务条件。角点能被割刀扫到却不被中心经过时，只可计入cutter_footprint，不应称为centre_exact。

对centre_exact模式，当前验证只使用精确直线轨迹、服务点和曲线端点作为可证明的中心服务，因而偏保守。它不会凭图上看起来靠近就算完成。

完整任务面积另算，分别报告原田块/声明作业区/预声明人工带/额外残留。所有mandatory目标被扫过，不代表几何面积全部覆盖。示例05故意展示这个区别。

## 七、动画与可视化学习顺序

先打开03 → First curve → 低倍速播放，观察车体位置随时间沿Bézier弯道变化，航向与切线对齐。再看events中的BEZIER5六控制点、profile(T/lambda等对应参数)、curve_path空间采样和trajectory_reference时间数据。

下一步打开04，不应出现TURN；圆弧前后的STEER表示车身不动但转舵机构改变角度。最后对比03/07同图同覆盖次序的原折线与曲线。05便于观察真正的Bezier曲率从0连续升高再回0。

断点推荐 `curve_planner._blend`、`CurveTimeOptimizer.solve`、`evaluate_curve_event`、`validate_curves`。调试规划最好使用CLI启动项；GUI计算在子进程，主进程断点不会自动进入worker。

理论接口参考在 [REFERENCES_V2.md](REFERENCES_V2.md)。当前证据只覆盖所列案例和测试，不外推到实车或所有地形。
