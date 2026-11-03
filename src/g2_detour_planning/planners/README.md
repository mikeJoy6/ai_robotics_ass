# planners —— 绕障规划器（一个文件一个，`@register` 自动发现）

接口（`_types.py`）：`PlanRequest`（ArmSolver、碰撞回调 `valid(q)`、q0、目标位姿、粗筛障碍点、"上"方向、时间预算 8s）→ `PlanResult`（关节路径、末端路点、说明、统计）。
纯计算，不碰 ROS、不发指令；全部在手臂链根系 `arm_base_link`。`__init__.py` 里 `DEFAULT = "via_then_rrt"`。

| 文件 | 内容 |
|---|---|
| `_edges.py` | ★ `EdgeChecker`：一段的验证（粗筛 → 积分笛卡尔 → 关节兜底 → 放开姿态 → 姿态引导 → path_sane → 整机碰撞）、目标检查（目标有不撞 IK 解才搜）、直线直达、姿态调度（起终点 slerp）、捷径平滑、统计文本。参数 `EDGE_DEFAULTS` |
| `via_points.py` | 绕行模板 V/梯形/U × 8 方向（"上"排第一）× 6 高度(8~42cm)，按路程排序逐个验证；`try_templates` 被混合规划器复用 |
| `via_then_rrt.py` | ★默认：模板 → 末端 RRT |
| `via_then_astar.py` | 模板(≤40) → 加权 A* |
| `ee_rrt.py` | 末端空间 RRT（起终点包围盒 +35cm，目标偏置 0.2，步长 10cm，固定种子可复现）；`rrt()` 被 via_then_rrt 复用 |
| `_grid.py` | 5cm 格子惰性 A* 家族共用件 |
| `astar_w25.py` / `astar_w1.py` / `astar_clear.py` / `theta_star.py` | 加权 A*(×2.5) / 标准 A* / 离障加罚 / 任意角度 Theta* |
| `joint_rrt.py` | 关节空间 RRT-Connect + 捷径（OMPL 式对照组） |
| `_registry.py` | 发现与注册 |

失败信息里的统计怎么读："验证 N 段（IK+碰撞 M 段）：粗筛拒/IK 拒/回环拒/碰撞拒，关节兜底/放开姿态/姿态引导 段数，碰撞查询次数"。
粗筛拒 0、碰撞拒很多、耗时满 8s → 多半是粗筛看不到障碍（虚拟碰撞体、附着物太大），见上级 README「踩过的坑」。
