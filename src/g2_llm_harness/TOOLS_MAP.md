# G2 能力地图:每种能力现在有哪些工具、哪个最好、还缺什么

2026-11-12 盘点。范围:本仓库 `src/g2_*` + `~/robot_harness`(LLM harness)+ `~/agi@g2_robot`(原项目)+ `~/sam3_service`。
“最好”只按已有实测下结论,没测过的写“未验证”。

## 总表

| 能力 | 候选 | 现在最好 | 依据 | 缺口 |
|---|---|---|---|---|
| 认出“是什么” | HSV 颜色;SAM 3 文字分割;G1 点云聚类(无语义,未移植) | **SAM 3** | 录的真机帧上积木/笔/包/手机都对,0.32 s/图 | 相近颜色串、泛称不稳;没做跨短语去重;单帧 |
| 算“在哪”(3D) | 掩码/色块 + 深度中位数 → base_link | **SAM 3 掩码 + 深度中位数**(`sam3_objects`) | 反投影和掩码中心差 < 1 px | 只有中心点,**没有朝向、尺寸、顶面**;没有多帧融合 |
| 物体指代(模型怎么说“那个”) | 颜色单槽位 `self.target`;编号表 `self.objects` | **编号表** | `find_objects` → `phone_1` → `look_at` / `approach` 已打通 | 颜色定位还是单槽位;编号不跨指令保留 |
| 头部对准 | `HeadAim`(URDF FK + 外参) | **HeadAim** | 用 URDF 离线残差 < 0.1° | **真机未测**;低头只到 30°,近处桌面够不到;不会用腰帮忙 |
| 手臂终点 IK | 4 种(I1′~I4) | **`g2_seeded_lm`**(`ArmSolver.ik` + `polish`) | 30/30、18 ms、关节转得最少(`../g2_arm_ik/`) | — |
| 手臂直线 | 积分笛卡尔 `plan_cartesian` | **积分笛卡尔** | 构型连续,真机误差 0 mm | — |
| 手臂绕障 | via_then_rrt / via_points / A* 家族 / ee_rrt / joint_rrt(OMPL 式) | **`via_then_rrt`** | 183/196,中位 0.58 s(`../g2_detour_planning/`) | 环境里没有桌子(见“避障环境”) |
| **手臂末端姿态** | `full`(锁死 6 维)/ `axis`(只对准工具轴,5 维)/ `pos`(只管位置) | harness 目前**只用 `full` 且保持当前姿态** | 真机:手机在右前方能到;往中间靠就可能够不到 | **要开放 `axis`(工具朝下、绕轴自由)和 `pos` 兜底**,见下 |
| 关节组运动(腰/头/臂) | `WholeBody.plan/run` + `JointDetour` | **WholeBody** | 隔离 domain 打桩验证;真机 goto_pose 用过 | harness 里新工具真机未测 |
| 避障环境 | octomap(头部点云,`../g2_head_pointcloud/`);planning scene 碰撞体 | **octomap**(设计上) | 15 道垃圾点过滤 | harness 按 `use_head_cloud:=false` 跑,**桌子不在碰撞里**;没自动加桌面碰撞体 |
| 末端执行器 | `suction_control`(EVS08 吸盘)、`task_board_pose` 的吸取流程 | 吸盘(在 agi@g2_robot) | 纸板任务跑通过 | **harness 没有抓/吸工具** |
| 移动底盘 | `genie_robot_agibot_g2_slam` | 未评估 | — | harness 没有 |
| 大模型 | DeepSeek `deepseek-flash`(工具调用) | 唯一在用 | 真机指令跑通 | 没有评测集(`scenarios/` 空),换模型无法量化比较 |
| 安全 | gateway 限位/范围、只规划、人按 y、次数上限 | gateway | 87 个离线测试 | 当前 `harness.yaml` 被放宽,越界目标要到规划器才被拒 |

## 用户指出的问题:末端只会平移

`approach_object` / `move_arm_end` 调 `plan_arm_to(arm, target, rpy=None)`:**终点姿态 = 当前姿态**(`full` 模式锁 6 维)。
手机在右前方时够得到;往身体中线靠,同一个手腕姿态下 IK 就可能无解,直线和绕障都失败。

已有但没接上的能力:

- `ArmSolver` 本来就有 `axis`(只要求末端 X 轴对准一个方向,绕它转自由,5 维)和 `pos`(只管位置)模式;
  `plan_cartesian(arm, p, R, mode)` 也接受 `mode`。
- 绕障的中途路点已经会“先 axis、再 pos”放松(`planners/_edges.py`),但**终点**还是 `full`。

建议的顺序(由松到紧,都要过碰撞检查):

1. 悬停类任务(“到手机上方 30 cm”)用 **`axis` + 工具轴朝下**:姿态只约束“朝下”,手腕绕竖直轴随便转 → 可达范围明显变大。
2. 还不行退到 **`pos`**(只要到位置),结果里告诉模型姿态没保证。
3. 再不行让模型**用腰**:`move_joints(body)` 转腰 / 前倾,再试一次(5 号关节转腰正 = 向左)。
4. `plan_detour` 要加 `mode` 参数,终点也能用 `axis` / `pos`。

## 缺口,按优先级

1. **手臂终点姿态放松**(上面一节)。直接决定“能不能到”,代码现成,改动小。
2. **桌面进碰撞检查**:开 `use_head_cloud:=true`(用 `../g2_head_pointcloud/`),或用 RANSAC 拟合桌面后往 planning scene 加一个盒子。现在绕障看不见桌子。
3. **物体几何**:SAM 3 掩码里的点做主轴 / 顶面 / 尺寸(G1 `fps_geometry` 的做法)。抓取、选接近方向都要。
4. **末端执行器工具**:把 `suction_control` 包成 `suction_on/off/status`,`status` 以实测 `attached` 为准;`suction_off` 永远人工确认(会掉东西)。
5. **跨短语去重 + 多帧**:同一块掩码被两个短语认领时留分数高的;关键动作前取 3~5 帧做中位数和稳定性检查(颜色定位已经这样做)。
6. **评测集**:`scenarios/` 写 20~30 条指令 + 假机器人期望工具序列,换模型、改提示词后跑一遍看成功率。
7. **编号表统一**:颜色定位也进编号表(`red_1`),淘汰单槽位 `self.target`。
8. 底盘、整机抓放流程:等上面做完再评估。
