# g2_detour_planning —— 手臂末端空间绕障规划 + 执行（AgiBot G2，替代 OMPL）

> 原样复制自 `~/agi@g2_robot/src/genie_robot_agibot_g2_motion/`（运控包）和 `genie_robot_agibot_g2_hardware/`（测试工具），2026-11-03。
> **没改过 import**：`from .algorithms`、`from .kin`、`from .ik_lab` 在 `../g2_arm_ik/`，
> `genie_robot_agibot_g2_perception.octomap_util` / `voxel_util` 在 `../g2_head_pointcloud/`。要跑就把三处放回原包结构。

## 为什么不用 OMPL

OMPL 在关节空间随机采样，路线会"末端走一点、胳膊抡一大圈"，手也不受保护。同题离线对比（seed 1 留出集）：
OMPL 式 `joint_rrt` 成功率最高，但 **43% 的路径让手扫过障碍**、臂离障只有 1.4cm、路程 1.55×直线；
末端规划器路程 1.39×、臂离障 2.7cm、手扫过障碍 5%。

## 结论：默认 `via_then_rrt`

先试全部绕行模板（V / 梯形 / U × 8 方向 × 6 高度，按末端路程从短到长，第一个通过的就是模板里最短的），都不通再在剩余预算里跑末端 RRT 兜底，
最后捷径平滑成几段直线。三组留出题合计 **183/196** 最好（via_points 182、via_then_astar 180，差在噪声内），规划中位 **0.58s**，预算 8s。

| 规划器 | 思路 | 评价 |
|---|---|---|
| **via_then_rrt** ★默认 | 模板 → 末端 RRT | 最稳 |
| via_points | 只有模板 | 几乎一样好，模板想不到的形状（L 型通道、两侧都有东西）解不了 |
| via_then_astar | 模板(≤40) → 加权 A* | 第一轮默认，新的段验证下 A* 变弱 |
| ee_rrt | 末端 RRT（目标偏置 0.2，步长 10cm）+ 捷径 | 慢，15% 超 3s |
| astar_w25 / astar_w1 / astar_clear / theta_star | 5cm 格子 A* 家族（`_grid.py`） | 每段贵、8s 里验证的段少；w1（"最优"）几乎全超时 |
| joint_rrt | 关节空间 RRT-Connect = OMPL 式对照组 | 见上，用户不要 |

## 核心："一段"怎么验证（`planners/_edges.py`，所有末端规划器共用）

一段 = 从上一段的**实际**关节角出发，积分笛卡尔把末端走直线到下一个路点 → 构型全程连续，这是不抡大圈的根本保证。便宜的先做：

1. **末端粗筛**：末端沿线段每半格采样，离障碍点 < 10cm 就拒（起终点 12cm 内不筛，要能贴近物体）。障碍点 = octomap 体素 + planning scene 碰撞体表面采样点。
2. **积分笛卡尔**走这段（迭代上限按段长给，走不到的段几毫秒就放弃）。
3. 走不到 → **局部关节插值兜底**（末端偏离直线 ≤max(3cm,15%段长)）→ 中途路点再走不到就**放开姿态**（先只保工具轴，再只要位置）。
4. **姿态引导**（posture_pull=2）：7 自由度冗余，每步在零空间把构型往"目标处最近不撞 IK 解"拉——治"末端通了、肘撞了"。
5. `path_sane` 防大回环；**整机碰撞**每个积分点都查（MoveIt `/check_state_validity`，含 octomap；关节变化 >2° 再插值查）。

"放开姿态 + 关节直连 + 姿态引导"三样加上后，140 道绕障题 via_then_rrt 125 → **134**。

## 执行：沿路径变速（`arm_motion.cart_profile` / `cart_samples`）

末端匀速 0.15 m/s，哪里单关节会超 URDF 速度限位或 3 rad/s² 加速度，就只在那一段压末端速度（简化 TOPP：正扫限加速、反扫限减速）。
关节角沿弧长 5mm 重采样 + 3cm 平滑（平滑后整条复查碰撞，撞了退回不平滑）；发 JTC 的速度取夹持三次样条导数 → JTC 插出来 C2。
对比旧的"整条一个五次多项式"：执行中位 8.8 → 5.8s，关节加速度峰最大 2677 → 2.94 rad/s²。
坑：按 JTC **实际插出来的样条**量才看得出问题（拐角猛甩、起步跳速、C1 速度导致加速度跳）。

## 文件

```
g2_detour_planning/
├── arm_motion.py      ★ ROS 执行层：关节反馈、base_link↔手臂根系换算、MoveIt 碰撞检查、粗筛障碍点、
│                        plan_detour / plan_cartesian / pose_to、cart_profile 计时、发 JTC action、等停稳
├── planners/          规划器（一个文件一个，@register），见 planners/README.md
├── sim_world.py       离线碰撞世界：URDF 凸包 + SRDF 自碰撞豁免 + octomap 体素（~1ms/次，MoveIt ~2~4ms/次）
├── planner_bench.py   离线对比所有规划器（不用机器人/MoveIt）：成功率、规划/执行时长、路程、离障、峰值速度/加速度
├── pose_ab_search.py  真机 A/B 往返测绕障（Shift+A/B 录、a/b 去、c 循环、空格取消）
├── bench/             planner_bench 用的真机快照，见 bench/README.md
└── tools/             测试用虚拟障碍 / 附着物，见 tools/README.md
```

## 踩过的坑（改之前必须知道）

- **`/check_state_validity` 必须 `is_diff=True`**：`False` 会把附着碰撞体（手上拿的纸板）全清掉，挂着纸板插进料框照样报"不撞"（实测）。
- **粗筛看不到 MoveIt 虚拟碰撞体**（相机看不见、不在 octomap）→ 每个穿墙模板都要真走 IK+碰撞（~50ms/段）才被拒，8s 预算耗光：
  料框测试一题 9.4s 超时。`obstacles_root` 把碰撞体表面按 2.5cm 撒点并进粗筛后同题 0.6s。
- 粗筛只看末端一个点、离障 10cm：手上挂了大件（纸板半宽 ~15cm）时会放过一些真撞的路线，结果仍由整机碰撞兜住，只是变慢。
- 评估要大样本：56 道题噪声 ±3 道，seed 1 上调参在 seed 2 上反而变差；结论用 3 组留出题（196 道绕障）。
- 预算按"IK+碰撞的段数 ≤1500 或 8s"算；碰撞服务是瓶颈（每次 2~4ms，一题上千次）。
