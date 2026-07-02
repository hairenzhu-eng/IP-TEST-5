# Research Findings

## Research Question

LiDAR聚类、EKF运动预测和基于目标尺度的改进APF，能否在典型COLREG会遇场景中提高无人水面艇避碰安全性与行为可解释性？

## Current Understanding

当前系统把感知、预测、规则判别和控制组织成可追踪的流水线。LiDAR点云经DBSCAN形成目标簇，主成分尺寸PC1/PC2构成椭圆安全域；EKF估计目标运动，并通过TCPA/DCPA及虚拟碰撞点提前激活避碰；规则路由按相对方位、航向和速度区分追越、迎遇与交叉；改进APF将目标吸引、障碍排斥、COLREG侧向偏置和固定航速下降组合为航向命令。

证据最强的结论是EKF预测的重要性。现有消融结果中，关闭EKF后仅左向右交叉保持稳定，其余动态会遇普遍恶化。目标尺度感知在三组配对中正确拉开大型和小型目标的PC1中位数，但迎遇场景出现反转，说明视角、遮挡、点云密度和轨迹关联会影响几何估计。系统不能被描述为已经证明“完全符合COLREG”：追越场景全部失败，部分日志的场景分类也与环境名称不一致。

## Key Results

- 32条选择性消融轨迹覆盖4类会遇、2种目标尺度和4种EKF/尺度开关组合。
- 以图中原始判据计，左向右交叉表现最好；右向左交叉仅EKF+尺度组合稳定；迎遇有条件成功；追越全部失败。
- 大/小目标PC1中位数分别为：左向右1.46/0.86 m，右向左1.49/0.90 m，迎遇0.59/0.65 m，追越1.42/0.39 m。
- 当前代码的后处理阈值与部分已生成图标签不一致，因此论文不跨场景汇总单一“成功率”。

## Patterns and Insights

- 预测比单纯放大势场更关键：无EKF时，动态目标通常被当作静态障碍，COLREG会遇类型难以可靠触发。
- 尺度感知的价值主要体现在风险域自适应，而非保证每次都获得更大的最小间隙。
- 单一轨迹的几何成功不等同于COLREG合规；合规还要求正确分类、及早且明显的动作、保持/让路责任和可预测性。
- 追越失败与目标相对运动、分类漂移和近共线航迹有关，需单独重构触发与超越侧策略。

## Lessons and Constraints

- 不把Scoping Study中的Behavior Tree和GPMP2写成已完成实现。
- 不把“clearance > -0.30 m”称为物理无碰撞；负值表示等效边界重叠，只能解释为带容差的代理判据。
- 不从每格仅一次运行推导统计显著性。
- 不把环境文件名当作真实分类结果；日志中的APFEncounter必须同步核对。

## Open Questions

- 统一成功阈值后，重复随机种子下的成功率、最小间隙、路径增量和控制平滑性如何？
- 迎遇PC1反转由视角、聚类、轨迹关联还是船体几何造成？
- 如何为追越建立稳定的相对速度门限、船尾通过目标和退出条件？
- 如何把当前显式规则路由正式重构为Behavior Tree，并引入GPMP2作为远期平滑规划层？

## 2026-06-29 Citation and Manuscript Finalization

- The English manuscript now uses claim-level Harvard citations rather than relying on a long but partially dangling reference list.
- Five incorrect bibliographic identifiers were corrected and three incomplete records were enriched from CrossRef or publisher metadata.
- The COLREG schematic distinguishes Rule 13 overtaking, Rule 14 head-on, Rules 15-16 give-way crossing, and Rule 17 stand-on crossing without implying that the regulation prescribes exact software thresholds.
- The pseudocode is a conservative reconstruction of `src/laptop.py`: candidate obstacles are screened, classified, ranked by rule priority/TCPA/DCPA, and dispatched to crossing, overtaking/head-on, or default APF branches.
- The cited DOCX passes structural and accessibility checks. Full raster rendering remains blocked by the absence of LibreOffice and a repeatable Word COM export stall.

## Optimization Trajectory

本轮不重新调参。研究贡献来自对现有32条选择性消融结果的外层综合：从“证明完整合规”转为“界定预测与尺度感知的作用，并识别追越和验证协议的关键缺陷”。

## 2026-06-29 APF/EKF Theory Grounding

- The obstacle predictor is not a four-state constant-velocity filter. `ObstacleEKF` implements a five-state CTRV model `[p_x, p_y, v, heading, turn_rate]`, uses a numerical transition Jacobian, a position-only measurement matrix, and Joseph-form covariance correction.
- The size-aware APF replaces circular distance with an ellipse level derived from PC1/PC2 plus the own-vessel radius. Its cluster branch uses a clipped linear proximity weight, while the classical switch branch uses the finite-influence inverse-distance gradient.
- The newly added schematic separates the scalar potential interpretation from the controller's actual vector decomposition: goal/path attraction, observed and predicted repulsion, COLREG/side-lock bias, and the bounded resultant heading.
- The inserted theory is supported through Harvard author-year citations and introduces no new experimental claim.
