# 把这个已完成版本交给本地 Codex

本项目不需要Codex才能运行。先在GUI中完成03/04演示，再用Codex继续改进。根目录已提供AGENTS.md，描述当前真实架构与不能破坏的约束；这种项目指令文件机制见OpenAI官方文档：<https://learn.chatgpt.com/docs/agent-configuration/agents-md>。

在你已有的本地Codex界面中选择**完整项目根目录**，不要只选gui.py或只选examples。先建立Git基线（没有初始化过时再运行git init）：

```powershell
git init
git add .
git commit -m "Baseline: constrained curve motion v2"
git switch -c feat/next-improvement
```

首次Git提交可能提示本机尚未设置用户名/邮件，按你自己的Git身份配置，不要复制他人身份。Git命令在VS Code终端执行，下面prompt在Codex聊天输入框执行。

## 第一个可复制prompt：只阅读与验收

```text
请先阅读根目录AGENTS.md、START_HERE_中文.md、README_中文.md和
 docs/TRAJECTORY_DESIGN_中文.md，检查实际代码，不要修改任何文件。
使用本项目虚拟环境运行pytest，并分别对examples/demo_03和demo_04进行独立验证。
请解释原始legs→曲线原语→时间参数化→evaluate_events→GUI车体动画的调用链。
明确区分几何G2、时间连续性、最小转弯半径、原地TURN、静止STEER和闭环跟踪。
列出你实际执行的命令和结果；未运行GUI或Windows测试就明确写未运行。
最后给出最多三个后续改进选项，不要直接写代码或更改示例数据。
```

## 后续窄任务模板

```text
本轮只解决【一个具体问题】。
复现条件：预设【03/04/05/06】，参数【...】，操作【...】，观察到【...】。
预期：在【约束】不变的条件下得到【可以测量/观察的结果】。
不能降低R_min、删required目标、绕过全车体碰撞验证或给前进模式插入TURN。
先定位原因和拟修改文件，给出回归测试；再做最小修改。
最后跑pytest、重验03/04，并说明UI、事件、CSV、验证是否仍使用同一条轨迹。
```

合适的下一项研究是：**在连续曲率的G2相邻原语之间，联合分配非零边界速度**。这要改变时间模型并重新证明或保守检查导数与转舵约束，不能只提高播放倍速或改位置插值；也不能把现有“每段静止”说成全局时间最优。

另一个独立方向是加入理想自行车/Pure Pursuit闭环跟踪，但必须把“reference”和“actual simulated state”画成不同轨迹、记录跟踪误差和执行约束，再说明它仍不是实车。当前v2尚未实现这个控制器。
