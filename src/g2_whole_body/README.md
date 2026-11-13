# g2_whole_body —— 腰 / 头 / 双臂关节组运动:碰撞检查 + 绕行 + 4 个 JTC(AgiBot G2)

> 原样复制自 `~/robot_harness/harness/motion/`、`scripts/goto_pose.py`、`configs/poses.yaml`(2026-11-12,未提交)。
> `whole_body.py` 最早来自 `~/agi@g2_robot/src/task_board_pose`,在 robot_harness 里拆成了 `plan()` + `run()`。**没改过 import**。

## 结论:用哪个

| 场景 | 用 |
|---|---|
| 某一组关节转到目标角(腰、头、单臂) | `WholeBody.plan(target)` → 人确认 → `WholeBody.run(plan)` |
| 直线插值会撞 | `JointDetour`:先经 `poses.yaml` 里录好的位姿中转,不行再关节 RRT |
| 全身回到人录的位姿 | `scripts/goto_pose.py <名字> [--groups head]` |

手臂**末端**的直线/绕障不在这里,见 `../g2_detour_planning/`(`via_then_rrt`)和 `../g2_arm_ik/`。

## 怎么工作

- 关节组 = `hardware.launch` 起的 4 个 JTC(`joints.py` 的 `GROUPS`):`body` 5 个(idx01~05)、`head` 3 个(idx11~13)、`left_arm`(idx21~27)、`right_arm`(idx61~67)。
- `plan()`:当前角 → 目标角线性插值,每 `check_step` 弧度调一次 MoveIt `/check_state_validity`(整机,含 octomap);会撞就交给 `detour`;
  时长按最慢关节的峰值速度算(`speed`),不少于 `min_duration`。返回 `JointPlan`(或错误字符串)。
- `run()`:**执行前**检查机器人离规划起点 > 0.05 rad 就拒绝(规划和执行之间人可能动过它);各组 JTC 同时发,等停稳。
- 规划和执行分开,是为了中间能让人看一眼按 `y`(harness 的 gateway 就是这么用的)。

## 关节含义(从 URDF 读出,给大模型的工具说明要写进去)

| 组 | 关节 | 轴 / 正方向 |
|---|---|---|
| body | 1、2、3 | 绕 y 俯仰,三者之和是上身前倾;1、2 主要改高度,前倾优先改 3 |
| body | 4 / 5 | 绕 x 侧倾 / 绕 z 转腰(正 = 向左) |
| head | 1 | 绕 z 左右(正 = 向左,±44.7°) |
| head | 2 | 绕 x 歪头(±8.7°) |
| head | 3 | 绕 y 点头(正 = 低头,−1°~30°) |

## 文件

```
g2_whole_body/
├── joints.py         GROUPS:组名 → (JTC 名, 关节名列表);parse_groups
├── whole_body.py     ★ WholeBody:snapshot / _valid(碰撞)/ check_path / plan / run / cancel;JointPlan
├── joint_detour.py   JointDetour:经录好位姿中转 → 关节 RRT
├── poses.yaml        人录的全身位姿(look 等),也是 JointDetour 的中转候选
└── scripts/goto_pose.py  全身去某个位姿,打印每个关节变化,按 y 才动
```
