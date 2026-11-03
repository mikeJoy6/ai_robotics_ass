# ik_lab/algorithms —— 可插拔 IK 算法（一个文件一个，`@register` 自动发现）

加算法：新建一个模块，类上写 `@register('<名字>', '<说明>')`，继承 `Algorithm` 实现 `plan(request)` 返回 `CandidatePath` 列表；
不用改任何中心列表（`_registry.discover()` 扫描本目录）。约定见 `_types.py`：只拿起点关节 + 目标 TCP 位姿，不碰 ROS、不执行。

| 注册名 | 文件 | 思路 | 结论（G2 右臂 30 个随机远目标） |
|---|---|---|---|
| **`g2_seeded_lm`** | `g2_seeded_lm.py` | 调 `../../algorithms.py` 的 `ArmSolver.ik`（q0+60 随机种子、零空间拉近、最近且不撞）再 LM 精修到 0.1mm | **最优**：30/30，平均 18ms、最长 26ms，转角最少 |
| `multi_ik_adaptive` | `multi_ik_lm.py` + `_adaptive_ik.py` | 确定性种子，单种子换成自适应 LM | 30/30，最长 93ms |
| `multi_ik` | `multi_ik_direct.py` | 确定性种子（`_seeds.py`）+ 固定小步 DLS；收 5 个解、查整条直线、选行程最小 | 21/30 |
| `ik` | `ik_direct.py` | 3 个种子 + `solve_pose` + 关节直连 | 2/30，基本不可用 |
| `joint` | `joint_direct.py` | 不是 IK：回放录制的关节角，对照基线 | — |

共用件：`_types.py`（请求/候选/接口）、`_registry.py`（发现与注册）、`_seeds.py`（确定性种子：当前、关节中点、零空间偏移、单关节偏移，按 FK 离目标排序）、
`_adaptive_ik.py`（自适应 LM 本体：信赖域 + 真实误差回退 + 自适应阻尼）。

设计文档（英文，genie 原版）：`multi_ik_direct.md`（多解直连流程逐步构造）、`multi_ik_adaptive.md`（从固定小步 DLS 到自适应 LM 的推导与参数）。
