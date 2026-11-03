# g2_arm_ik —— 7 自由度手臂 IK 与积分笛卡尔（AgiBot G2）

> 原样复制自 `~/agi@g2_robot/src/genie_robot_agibot_g2_motion/genie_robot_agibot_g2_motion/`（2026-11-03），**没改过 import**：
> 文件里的 `from .kin`、`from ...algorithms` 等按原包结构写，单独放在这里跑不起来。要用就整体放回一个 Python 包
> （包名随意），目录结构保持本文件夹的样子即可。无 `package.xml`，colcon 不会构建本目录。

## 结论：用哪个

| 场景 | 用 | 位置 |
|---|---|---|
| **解终点 IK（首选）** | **I1′ = `ArmSolver.ik` + `polish`**（`ik_lab` 里注册名 `g2_seeded_lm`） | `algorithms.py` |
| 末端走直线、构型连续不抡大圈 | C1 积分笛卡尔 `ArmSolver.solve` + `limit_step` + `path_sane` | `algorithms.py` |
| 键盘点动（每按一次走一小步） | C2 `solve_pose` | `ik_lab/kinematics.py`、`differential_ik.py` |

四个 IK 同场对比（离线 G2 右臂，30 个随机远目标，1s 预算）：

| 算法 | 成功 | 平均 | 最长 | 关节总转角中位 |
|---|---|---|---|---|
| I2 `ik`（3 种子 + solve_pose） | 2/30 | 29ms | 36ms | 277°（只 2 个样本） |
| I3 `multi_ik`（多种子 + solve_pose） | 21/30 | 198ms | 329ms | 388° |
| I4 `multi_ik_adaptive`（多种子 + 自适应 LM） | 30/30 | 18ms | 93ms | 365° |
| **I1′ `g2_seeded_lm`** | **30/30** | **18ms** | **26ms** | **341°** |

I1′ 最优：成功率并列第一、最长耗时最稳（26ms vs 93ms）、关节转得最少（零空间拉近的效果）。
mock 往返实测执行也最短（I1′ 4.94/5.03s vs I4 5.79/5.67s）。另测：固定起点 → 20 个随机可达位姿，左右各 20/20，平均 17ms。

## 算法原理

- **I1′（`ArmSolver.ik` → `polish`）**：q0 + 60 个均匀随机种子；full 姿态模式单种子用 KDL LMA（C++，快），axis/pos 模式用自适应 LM；
  合格解按离 q0 欧氏距离排序，最近 8 个沿**零空间往 q0 拉近**（末端不动、关节靠近当前构型），给了碰撞回调就取最近且不撞的；
  最后自适应 LM 精修 ≤30 轮，3mm → 0.1mm。
- **自适应 LM**（`ArmSolver.lm`，源头 `ik_lab/algorithms/_adaptive_ik.py`）：误差 `[位置; 0.2·姿态]`、信赖域限步长（初始 0.25rad）、
  按**真实 FK 误差**回退验收、实际/预测下降比自适应调阻尼和半径、限位上往外推的关节冻结、连续 6 次不降就放弃（坏种子几步就丢）。
- **C1 积分笛卡尔**：从实测构型出发，每步末端误差限到 ≤1cm/0.03rad，DLS(λ=0.05) `dq = J⁺e`，每步重算 FK 闭环，收敛 3mm/0.02rad。
  每步都很小 → 相邻构型连续 → 结构上不换分支。撞限位还往外推的关节锁住、其余接管。
  `path_sane` 兜底：单步关节 ≤0.1rad、总行程 ≤30×末端直线距离(rad/m，转角按 0.2m/rad 折进距离)、终点不换分支。
- **I3/I4**（genie 框架）：确定性种子（当前、关节中点、零空间 ±0.35/0.7/1.0、单关节 ±0.45/0.9，按 FK 离目标排序），
  收 5 个互差 ≥3° 的解，每个解的整条关节直线每 2° 查碰撞，选 Σ行程+0.5·max 最小。I3/I4 只差单种子求解器。

算法来源链（谁抄谁）：积分笛卡尔 tiangong `differential_ik.py` → sanofi `grasp_server._solve`；
自适应 LM genie `_adaptive_ik.py` → sanofi `grasp_server._ik`；本目录的 `algorithms.py` 是 sanofi 版拆成无 ROS 的 `ArmSolver`。

## 文件

```
g2_arm_ik/
├── kin.py              URDF 字符串 → KDL 链：FK / 雅可比 / 阻尼伪逆 / KDL LMA。G2 链 arm_base_link→arm_{l,r}_end_link
├── algorithms.py       ★ ArmSolver：C1 solve、I1 ik/lm/pull_near、polish；常量全在文件头（无 ROS，可离线单测）
├── ik_lab/             IK 对比框架（genie 移植，@register 插件式）+ C2 solve_pose，见 ik_lab/README.md
├── differential_ik.py  键盘笛卡尔点动（ROS 节点，发 JTC），用 C2
├── pose_ab.py          A/B 位姿往返测 IK（ROS 节点，--algorithm 选 ik_lab 里的算法，默认 g2_seeded_lm）
└── test_algorithms.py  离线单测（读 G2 URDF；含一项 arm_motion.cart_profile 的测试，那个文件在 ../g2_detour_planning/）
```

## G2 专有事实

- 手臂链根 `arm_base_link` 相对 `base_link` 绕 X 转了 −90°；对外接口一律 base_link 系，内部用**实测腰关节** FK 换算后再解。
- `kin.py` 静音了 urdf_parser_py 的 "Unknown attribute" 告警（G2 URDF 有 `collision name` / `collision_checking` 扩展字段）。
- 没搬 sanofi 的 `band` 姿态模式（抓垃圾袋专用）、genie 的 `multi_ik_5d` / `link6_ik`（按 G1 关节名写死）。
