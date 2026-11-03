# ik_lab —— IK 对比框架（genie 移植）+ 共用运动学小件

来源：`g1_pointcloud_grasping/src/genie_motion/genie_motion/`（G1 项目），搬到 G2 时只去掉了按 G1 关节名写死的 5D 打分。
`ChainKin` 用的是上一级的 `../kin.py`，所以 ik_lab、键盘点动、arm_motion 共用同一个 URDF→KDL 模型。

| 文件 | 内容 |
|---|---|
| `kinematics.py` | `solve_pose`（**C2**：固定小步 DLS 局部 IK，每步 ≤2mm/0.01rad，收敛 0.1mm/0.001rad，≤100 轮）、`rotation_vector` / `rotation_about` / `IkResult` |
| `pose_task.py` | 姿态残差、姿态雅可比（精确 `Jr⁻¹·Jω`）、任务零空间（生成种子用）、关节行程权重 |
| `planning.py` | `MotionSettings` + `make_trajectory`：两点五次多项式时间参数化（速度/加速度/加加速度限），`plan_motion` 跑算法并校验候选路径。`arm_motion.pose_to` 的点到点计时也用它 |
| `algorithms/` | 可插拔 IK 算法，见 `algorithms/README.md` |

调用方：`../pose_ab.py`（A/B 往返测 IK）、`../differential_ik.py`（键盘点动用 `solve_pose`）。
