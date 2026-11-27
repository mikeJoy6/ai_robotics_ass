# G2 手臂逆运动学

同一个夹爪位姿，七轴手臂往往能以好几种肘部姿势到达。只给出目标的**终点逆解**时，关节怎样走到那里仍是一个独立问题。如果希望夹爪从当前位置沿直线推进，就要逐步求解，并保留沿途的关节角。本目录收录这两类解法，以及比较它们的代码。

[仓库首页](../../readme.md) · [IK 对比框架](ik_lab/README.md) · [算法索引](ik_lab/algorithms/README.md) · [手臂绕障规划](../g2_detour_planning/README.md)

## 从任务选算法

| 想解决的问题 | 方法与输出 |
| --- | --- |
| 给定目标位置和完整朝向，找一个离当前手臂较近的构型 | [g2_seeded_lm](ik_lab/algorithms/g2_seeded_lm.py) 默认从当前构型和 60 个随机初值求逆解，把最近的几个解沿零空间往起点拉近。选中通过状态检查的终点后，用自适应 LM 精修，交给公共层检查整条关节直线。 |
| 夹爪沿直线接近目标，途中也需要关节路点 | [积分笛卡尔 ArmSolver.solve](algorithms.py#L167) 从实测构型出发，每步用阻尼最小二乘（DLS）把末端向目标推进，再计算正运动学（FK）修正下一步。返回整条关节路径、收敛标志、末端误差和最后构型。 |
| 夹爪微移或绕自身轴转一点 | [局部 IK solve_pose](ik_lab/kinematics.py) 从一个给定构型开始，对位置和姿态误差分量逐步限幅，返回新关节角和误差。[键盘点动调用方](differential_ik.py) 据此处理一次位移或转角。 |
| 比较少量初值和多初值能找到多少种构型 | [ik](ik_lab/algorithms/ik_direct.py) 默认取 3 个初值，用局部 DLS 生成候选。[multi_ik](ik_lab/algorithms/multi_ik_direct.py) 从更多确定性初值求至多 5 个不同解，检查通往各解的关节直线，选总转角最小的一条。 |
| 保留同样的选路规则，改善远目标的单次求解 | [multi_ik_adaptive](ik_lab/algorithms/multi_ik_lm.py) 以[自适应 Levenberg–Marquardt（LM）](ik_lab/algorithms/_adaptive_ik.py) 替换固定小步 DLS。按真实 FK 误差验收试探步，在收敛慢或接近限位时调整步长和阻尼。 |
| 已有目标关节角，需要回放对照 | [joint](ik_lab/algorithms/joint_direct.py) 将起点与录制关节角组成两点路径，作为对照，用来区分求逆解和回到已知构型的耗时。 |

## 读懂输入和检查条件

运动学以 `arm_base_link` 为手臂链根，以 `arm_l_end_link` 或 `arm_r_end_link` 为工具末端（TCP）。若目标来自 `base_link`，须先用五个腰关节的当前角度换算到手臂链根。底层 `ArmSolver` 的 `full` 模式约束位置和完整朝向，`axis` 约束位置和末端 X 轴方向，`pos` 仅约束位置。对比框架接收完整位姿。[KDL 链与 FK](kin.py) · [坐标换算调用方](../g2_detour_planning/arm_motion.py) · [对比框架请求](ik_lab/algorithms/_types.py)

- **沿路径跟踪。** 积分每步把位置和旋转误差向量的长度分别限到 1 cm、0.03 rad。`limit_step` 遇到将越界的关节会尝试锁住它，让其余关节补偿。位置误差小于 3 mm 且姿态误差小于 0.02 rad 时结束。`path_sane` 再检查单步关节变化（最多 0.1 rad）、整条路径累计转角，以及调用方提供参考终点时的分支一致性。总转角上限按等效末端距离的 30 倍计算，距离下限取 0.01 m。完整位姿调用方还把姿态变化按 0.2 m/rad 折算进距离，并另行采样检查碰撞。[积分与路径检查](algorithms.py) · [路径调用方](../g2_detour_planning/arm_motion.py)
- **只求终点。** `ArmSolver.ik` 在 `full` 下对每个初值调用 KDL LMA，再复核限位和 FK。对 `axis` 或 `pos`，当前构型用积分法，随机初值用自适应 LM。最近 8 个候选交给 `pull_near`，在保持目标容差的同时尽量减小关节变化。提供 `valid` 回调时再筛终点。框架插件随后精修到 0.1 mm、0.001 rad，公共层每隔至多 2° 采样关节直线路径。[终点搜索与拉近](algorithms.py#L198) · [插件精修](ik_lab/algorithms/g2_seeded_lm.py) · [路径验收](ik_lab/planning.py)

## 怎样理解对比结果

原项目留下的一组离线记录取 G2 右臂的 30 个随机远目标，每次规划预算 1 秒。两种 LM 方法都通过了全部目标，`g2_seeded_lm` 在这组样本中的最长耗时较短。各行耗时是历史记录，不能用来估计其他目标或机器的时延。

| 算法 | 成功数 | 平均耗时 | 最长耗时 | 成功样本关节总转角中位数 |
| --- | ---: | ---: | ---: | ---: |
| [ik](ik_lab/algorithms/ik_direct.py) | 2/30 | 29 ms | 36 ms | 277°（2 个样本） |
| [multi_ik](ik_lab/algorithms/multi_ik_direct.py) | 21/30 | 198 ms | 329 ms | 388° |
| [multi_ik_adaptive](ik_lab/algorithms/multi_ik_lm.py) | 30/30 | 18 ms | 93 ms | 365° |
| [g2_seeded_lm](ik_lab/algorithms/g2_seeded_lm.py) | 30/30 | 18 ms | 26 ms | 341° |

源码取自 2026-11-03 的 G2 运动包，保留了原项目的 Python 导入路径。积分求解沿用天工差分 IK 经 Sanofi 移植的实现。多种子对比框架源自 G1 的 `genie_motion`。A/B 程序展示怎样录两个位姿并用同一种算法往返，离线用例展示怎样用外部 G2 URDF 生成可达目标、通过 FK 复核关节解。[A/B 调用流程](pose_ab.py) · [离线用例](test_algorithms.py)
