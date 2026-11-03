# tools —— 往运行中的 MoveIt 加虚拟障碍 / 附着物（测绕障用）

原在 `genie_robot_agibot_g2_hardware/scripts/` 和 `launch/`，`ros2 run genie_robot_agibot_g2_hardware <脚本>.py` 调用。

| 文件 | 用途 |
|---|---|
| `add_collision_mesh.py` | 把一个 STL 作为 world 碰撞体加进 planning scene（`/apply_planning_scene`），同 id 再发 = 替换（改位置不用重启）。参数 mesh/frame/xyz/rpy/scale/object_id，xyz/rpy 写成字符串 `"'[0.8,0,-0.06]'"` |
| `attach_board.py` | 给末端挂/取纸板附着碰撞体（AttachedCollisionObject，不改 URDF）：0.59×0.405×0.02m 贴在吸盘端面，吸附点在中心下方 `--offset`；`--remove` 取下并从场景删除 |
| `hardware_obstacle.launch.py` | include 真机 `hardware.launch.py` + 用 add_collision_mesh 放一个料框 STL（`obstacle_xyz` / `obstacle_rpy` 参数） |

要点：
- 虚拟碰撞体相机看不到：MoveIt 碰撞检查会算它，但 octomap 里那片是空的；规划器粗筛要另外读 planning scene 的碰撞体（`arm_motion.obstacles_root` 已做）。
- 附着物挂上后，MoveIt 整机碰撞把它当手臂一部分，octomap 更新也会把它从点云滤掉；但调用方的 `/check_state_validity` 必须 `is_diff=True`，否则附着物被清掉。
- 纸板碰撞厚度 2cm 的理由：关节每 2° 查一次，末端外 ~1m 一步最多 ~3.8cm；2cm + octomap 体素 3cm > 3.8cm，采样之间穿不过体素。
- `attach_board.py --remove` 在同一个 diff 里先从手上取下（MoveIt 会把它放回 world）再从 world 删。
