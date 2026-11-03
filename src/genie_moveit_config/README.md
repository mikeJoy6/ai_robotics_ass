# genie_moveit_config — MoveIt / ros2_control 配置（数据基座）

> 框架层级：**数据基座**。见工作空间根 [../../../README.md](../../../README.md) §3「数据基座」。
> **★MoveIt Setup Assistant 自动生成，照抄自现有，勿手工合并进别的包★**（会被工具链再生成覆盖）。

## 装什么

```
genie_moveit_config/
├── config/
│   ├── A2D.urdf.xacro            顶层 xacro：include 结构 URDF + ros2_control 段
│   ├── A2D.ros2_control.xacro    ★ros2_control 段：<plugin>genie_hardware/GenieArmHardware</plugin>
│   │                              + HAL/WBC 话题名 + 每关节初值
│   ├── A2D.srdf                  规划组（left_arm/right_arm/body/head）、碰撞矩阵
│   ├── ros2_controllers.yaml     控制器定义（joint_state_broadcaster + 各组 controller）
│   ├── kinematics.yaml / joint_limits.yaml / pilz_cartesian_limits.yaml
│   ├── moveit_controllers.yaml / initial_positions.yaml / moveit.rviz
│   └── robot_description_kinematics_params.yaml
└── launch/                       move_group / rviz / rsp / spawn_controllers ...
```

## 关键交叉引用（动这些会断链）

- `A2D.ros2_control.xacro` 里 `<plugin>genie_hardware/GenieArmHardware</plugin>` —— 必须与
  `genie_robot_agibot_g1/genie_hardware.xml` 的 `<class name=...>` **逐字匹配**（pluginlib 按类名解析）。
- `A2D.urdf.xacro` 用 `$(find genie_description)/a2d_urdf/A2D.urdf` 引结构 → 依赖 `genie_description`。
- 本包名 `genie_moveit_config` 被 `hardware.launch.py` 多处 `get_package_share_directory('genie_moveit_config')` 引用 → **不能改名**。

> 注：`config/robot_description_kinematics_params.yaml` 顶部注释里写的 `ros2 run genie_hardware arm_control ...`
> 是旧单体的运行示例，已过时（本阶段不搬 `arm_control` 单体）。属生成文件，未改动，知悉即可。

## 性质

配置 + launch，不编译（仅 `install`）。换机器人 = 重跑 Setup Assistant 生成新的这一包。
