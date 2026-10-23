# 新增网页展示案例

先读 `PORTFOLIO_AUDIT_中文.md`。本轮结论与历史 v2.0.0 QA 分开记录。

GUI：用 `Open saved result...` 打开 `portfolio_examples/compact_greedy_astar`，点击 `First curve / turn`、1×、Follow，观察节点折线怎样转换成移动曲线。它是0.65×0.48m紧凑研究模型。加载后尺寸标签会更新，但编辑面板不会整体自动同步保存配置，重新规划应用保存的 config/map/robot。

展示真实 A* 绕障：打开 `portfolio_examples/obstacle_astar_bezier`。8条查询的直连确实被阻挡；完整查询在 queries.json。网页的 A* 图进一步区分展开顺序与返回路径顺序。

展示不同车辆：`mower_mst_astar` 是1.10×1.05m参考尺寸；`forward_dubins_r060/r090` 是禁止车体原地旋转的前进曲率受限模型。均为研究限制，不是品牌动力学标定。

公平算法对照：`compact_greedy_astar` / `compact_mst_astar` / `compact_ga_astar`。

节点访问顺序：每目录 `node_visit_order.csv/json`，包含重复访问。所有原始候选任务仍作服务义务；曲线模式割刀扫过即可服务，不保证中心精确过点。

复现用 `tools/build_portfolio_cases.py --out outputs/new_portfolio`；八组输入与完整保存结果均放在 portfolio_examples。素材生成用 tools/export_portfolio_assets.py，导出 MP4 额外需要 FFmpeg；没有 FFmpeg 可以使用已交付视频或 --skip-video。
