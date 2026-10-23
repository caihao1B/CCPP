# CCPP 2.1.0：先看这里

这是GUI2的新项目目录；不覆盖旧版。网站是另一个ZIP。

1. 完整解压，进入同时含app.py、ccpp_mowing、configs的根目录。
2. Windows已有Python3时，双击setup_windows.bat安装本目录独立.venv。
3. 双击run_gui.bat打开GUI，不要直接无参数运行包内__main__.py。
4. 默认选中08：方格GA、真实前进回环、无原地转向。点击Generate path + trajectory重算。
5. 也可Open saved result选择comparison_examples/square_ga；查看MST回访掉头请选square_mst，再点Heading loop，1×播放。
6. 09为三角格GA，10为方格MST，11/12为前进车0.60/0.90米半径。01—07标为旧基线，不要求无原地转向。
7. Turn treatment的loops_only要求完整移动曲线；找不到安全候选就失败，不瞬时改航向、不暗中缩小半径。原地转向仍可作为明确选择的其他模式。

完整PowerShell打开方式、逐条输入、VSCode运行与网页发布见docs/网页与GUI操作指南.md。

测试、对照数据和局限见docs/V2_1_测试与展示报告.md。原文档中v2.0.x测试与旧例子仅作为历史记录；当前214项测试、28+17项GUI检查、118项浏览器检查。

参考轨迹回放不是反馈控制仿真。曲线采用有限速度、加速度、jerk等约束；不含打滑/牵引/质量动力学。各基本段起止静止，前进车辆必要时停车转舵；不宣称全路径非零速度或全局最优。
