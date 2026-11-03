# ai_assistant —— 机器人开发知识库

## 本仓库内容

给 AI 读的机器人开发参考库：可直接照做的操作规程（`*.md`）+ 已经跑通的参考实现（`src/` 下的 ROS 2 包）。
目标：一个无上下文的 AI 读完后，能独立完成本仓库覆盖的机器人开发任务。


## 技术栈

| 项 | 值 |
|---|---|
| ROS | ROS 2 Humble |
| 运动规划 | MoveIt 2（`moveit_configs_utils`，OMPL，LMA/KDL 运动学插件） |
| 硬件接口 | ros2_control `hardware_interface::SystemInterface` 插件 |
| 构建 | `colcon`，`ament_cmake` |
| 机器人 | 智元 AgiBot G1（URDF 名 `A2D`）；文档中也给出 G2 的差异值 |

## 目录

```
ai_assistant/
├── README.md                                    ← 本文件
├── reference/                                   通用参考（与具体机器人无关）
│   ├── MARKDOWN_MATH_GUIDE.md / _EN.md          写公式文档的格式规范
│   ├── check_md_math.py                         公式格式检查脚本
│   ├── CapturePointSteppingDesign.md            公式文档正确样例（倒立摆 / capture point 推导）
│   └── 第一课教案_欧拉角的导数为什么不是角速度.md   公式文档错误样例（\( \[ 定界符，VSCode 不渲染）
└── src/
    ├── genie_moveit_config/                     MoveIt 配置包（G1 参考实现）
    │   ├── MOVEIT_SETUP_ASSISTANT_POSTPROCESS.md / _EN.md   Setup Assistant 产物后处理规程
    │   ├── README.md                            本包文件结构与交叉引用
    │   ├── config/                              A2D.urdf.xacro / A2D.ros2_control.xacro / A2D.srdf / *.yaml
    │   └── launch/                              Setup Assistant 生成的 launch（demo、move_group、rviz ...）
    ├── genie_robot_agibot_g1_hardware/          G1 真机 ros2_control 硬件插件
    │   ├── genie_hardware.xml                   插件注册：类名 genie_hardware/GenieArmHardware
    │   ├── src/arm_control_ros.cpp              插件实现（读 HAL/WBC 话题，平滑下发关节命令）
    │   ├── launch/hardware.launch.py            真机启动：ros2_control + 控制器 spawner + move_group + rviz
    │   ├── launch/hardware_head_off.launch.py   同上，但 control_head:=false（头部交给外部控制）
    │   └── scripts/force_monitor.py             六维力传感器终端监视工具（诊断用）
    │
    ├── g2_arm_ik/                               7 自由度臂 IK 与积分笛卡尔：4 种 IK 对比，最优 = g2_seeded_lm（ArmSolver.ik + polish）
    ├── g2_detour_planning/                      末端空间绕障规划(替代 OMPL，默认 via_then_rrt) + 沿路径变速执行 + 离线 bench + 虚拟障碍/附着物工具
    └── g2_head_pointcloud/                      头部深度图 → MoveIt octomap，15 道垃圾点过滤 + 背面补全 + 零星格清扫
```

`g2_*` 三个目录是 G2 项目（`~/agi@g2_robot`）的精选资产：原样复制、未改 import，没有 `package.xml`，colcon 不构建；每个目录都有自己的 README。

## 任务路由

| 任务 | 先读 | 参考实现 |
|---|---|---|
| 为新机器人生成 MoveIt 配置并接入真机 | `src/genie_moveit_config/MOVEIT_SETUP_ASSISTANT_POSTPROCESS.md`（英文：`_EN.md`） | `src/genie_moveit_config/`、`src/genie_robot_agibot_g1_hardware/` |
| 修改 / 理解 G1 的 MoveIt 配置 | `src/genie_moveit_config/README.md` | 同目录 |
| 写一个新的 ros2_control 硬件插件 | `src/genie_robot_agibot_g1_hardware/src/arm_control_ros.cpp`、`genie_hardware.xml`、`package.xml` 的 `<export>` | 同目录 |
| 写或修改含数学公式的 `.md` | `reference/MARKDOWN_MATH_GUIDE.md`（英文：`_EN.md`），完成后运行 `reference/check_md_math.py` | `reference/CapturePointSteppingDesign.md` |
| 平衡 / 步态控制的推导 | `reference/CapturePointSteppingDesign.md` | —— |
| 手臂 IK / 末端直线运动（选哪个算法） | `src/g2_arm_ik/README.md` | `src/g2_arm_ik/algorithms.py` |
| 手臂绕障规划（不用 OMPL）、轨迹时间参数化 | `src/g2_detour_planning/README.md` | `src/g2_detour_planning/planners/`、`arm_motion.py` |
| 深度相机建 MoveIt 避障地图、删垃圾点云 | `src/g2_head_pointcloud/README.md` | `src/g2_head_pointcloud/head_depth_cloud.py` 文件头 15 条 |
| 查看六维力数据 | `src/genie_robot_agibot_g1_hardware/scripts/force_monitor.py` 文件头注释 | —— |

## 关键事实（跨文件约束，改动前必须知道）

1. 插件类名 `genie_hardware/GenieArmHardware` 同时出现在 `genie_hardware.xml` 的 `<class name>` 和 `A2D.ros2_control.xacro` 的 `<plugin>`，两处必须逐字一致。pluginlib 按类名查找，与包名无关。
2. 包名 `genie_moveit_config` 被硬件 launch 通过 `get_package_share_directory('genie_moveit_config')` 引用，不得改名。
3. 硬件插件 `on_init()` 读取的每个 `hardware_parameters` key 都必须在 `A2D.ros2_control.xacro` 中有对应 `<param>`，缺一个即 FATAL。
4. 插件每周期最多移动 `STEP_SIZE = 0.01 rad`，所以关节最高速度 = `0.01 × update_rate`（G1：500 Hz → 5 rad/s）。
5. 控制器名必须在 `moveit_controllers.yaml`、`ros2_controllers.yaml`、硬件 launch 的 spawner 三处一致。
6. `joint_limits.yaml` 的数值必须写成浮点（`1.0` 而不是 `1`），否则 move_group 报参数类型错误。

## 行为规则（对所有任务生效）

- 不得自行启动真机 launch（`hardware.launch.py`、`hardware_head_off.launch.py`）。插件激活后立即向电机下发命令；真机启动只能由在场的用户执行。你只能做 `xacro` 展开、`colcon build`、`ros2 launch ... --show-args` 这类不驱动电机的验证。
- 文档中没有给出的数值（关节限位、话题名、频率）：从源码或参考实现推导；推导不出来就问用户，不要编造。
- 文档成对维护：`X.md`（中文）与 `X_EN.md`（英文）内容等价。改一份必须同步另一份。
- 新增含公式的文档必须通过 `python3 reference/check_md_math.py <文件>`（`0 error(s)`）。
- 新增规程文档时沿用现有风格：面向 AI、祈使句、每步带可执行的 `CHECK` 命令、占位符用 `<name>` 并在开头列表说明。完成后在本 README 的「目录」和「任务路由」中登记。
# ai_robotics_ass
