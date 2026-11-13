# g2_llm_harness —— 大模型通过工具调用指挥 G2(Agent 循环 + Gateway 安全关卡)

> 原样复制自 `~/robot_harness/`(工作区,2026-11-12,**未提交**;最近提交 `6bd385c`),**没改过 import**:
> `from .motion.joints`、`from ..perception.calib` 等按原包 `harness/` 的结构写。感知在 `../g2_object_perception/`,
> 关节组运动在 `../g2_whole_body/`,手臂运控是 `../g2_detour_planning/arm_motion.py` + `../g2_arm_ik/`。
> 要跑就用原仓库 `~/robot_harness`(`python3 scripts/run_task.py`)。完整设计文档在原仓库 `docs/`。

## 结论:结构

```text
用户指令 → loop.run_agent ⇄ DeepSeek(OpenAI 兼容 tool calling)
              ↓ 模型只能选工具、填参数
           gateway.Gateway.execute   参数校验、坐标计算、限位/范围检查、只规划模式、人按 y
              ↓
           backends/ros.py RosBackend(真机)| backends/fake.py FakeBackend(离线测试)
```

三条原则:

1. **模型给不了绝对坐标**。物体坐标由感知算、存在 gateway 里;模型只传颜色名、物体编号(`phone_1`)、相对偏移、关节增量、录好的位姿名。
2. **安全规则写在代码里**(gateway),不写在 prompt 里。碰撞交给规划器,gateway 只挡单位写错、范围明显不对。
3. **所有运动:规划 → 检查 → (`--plan-only` 到此为止)→ 人按 `y` → 执行**。每条指令最多 `max_moves_per_command` 次。

## 工具(给模型看的,`tools/objects.py` + `tools/motion.py`)

| 工具 | 作用 | 动不动 |
|---|---|---|
| `locate_object(color, arm)` | HSV 颜色定位,同色多个选离 `arm` 末端最近的 | 否 |
| `find_objects(query)` | SAM 3 按英文物体名找,按离相机由近到远编号 `<名>_1…` | 否 |
| `get_left_end_pose()` / `get_robot_state()` | 末端位姿 / 全部关节角 + 两臂末端 | 否 |
| `approach_object(object_id?, arm, standoff_cm)` | 末端到物体正上方(直线 → `via_then_rrt` 绕障) | 是 |
| `move_arm_end(arm, offset_cm, rotate_deg)` | 末端相对平移/转动 | 是 |
| `move_joints(group, deltas_deg/angles_deg)` | 腰/头/左臂/右臂关节(`WholeBody` 碰撞检查 + 绕行) | 是 |
| `go_to_pose(name, groups)` | 去 `poses.yaml` 里人录的位姿 | 是 |
| `look_at_object(object_id)` | 只转头,让物体在画面中央(`HeadAim`) | 是 |

## 文件

```
g2_llm_harness/
├── loop.py             ★ run_agent:问模型 → 执行工具 → 结果回给模型;SYSTEM 提示词
├── llm.py              DeepSeek 接口(标准库 urllib,读环境变量里的 key)
├── gateway.py          ★ Gateway:每个工具一个 _handler;物体编号表 self.objects;_plan_and_run_joints / _confirm_and_run
├── geometry.py         rpy ↔ 矩阵、相对转动(纯 numpy)
├── colors.py           颜色表 colors.yaml 读写、由采样像素算 HSV 范围
├── tools/              给模型看的 JSON schema(工具名必须和 gateway 的 handler 名一致)
├── backends/ros.py     真机:ArmMotion + WholeBody + 头部 RGBD + SAM 3 客户端 + HeadAim;发布 TF target_object
├── backends/fake.py    假机器人,方法和 ros.py 一一对应(含假 SAM 3、假转头)
├── configs/harness.yaml  模型、运动上限、工作范围、视觉、SAM 3 参数
├── scripts/run_task.py 入口:--real 真机、--plan-only 只规划;每条指令写 traces/<时间>.jsonl
└── tests/              87 个离线测试(假机器人;HeadAim 那 2 个要 PyKDL + G2 URDF)
```

## 踩过的坑(改之前必须知道)

- **工具说明要写清关节含义**:第一版没写,模型把“腰往前弯 90°”平均分给腰的 5 个关节。现在 `tools/motion.py` 的 `GROUP_TEXT` 写了每个关节绕哪根轴、正方向。
- **“最近的”要按要动的那条手臂算**:颜色定位曾写死按左臂选,右臂被派去够左边的物体(规划失败)。现在 `locate_object` 带 `arm`,`approach_object` 发现手臂不一致就拒绝。
- **两种定位要打通**:`approach_object` 曾只认颜色定位的单槽位,`find_objects` 找到的 `phone_1` 用不上。现在填 `object_id` 就查编号表。
- **`harness.yaml` 这份是放宽过的**:`max_travel_m` 1.70、`workspace` x 上限 1.85、右臂 y 上限 1.20(原来 0.70 / 0.85 / 0.20)。按这份配置,`test_workspace_refused` 等 3 个测试会失败;放宽后越界目标会一路送到规划器才被拒。
- `use_head_cloud:=false` 启动时 MoveIt 没有 octomap,**桌子不在碰撞检查里**,绕障也绕不开它;`approach_object` 只靠“路径最低点比物体高 13cm”兜底。
- DeepSeek key 只放 `.env`(gitignore,chmod 600),`llm.py` 从环境变量读。
