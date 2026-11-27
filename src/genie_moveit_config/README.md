# G1 运动规划配置 · 从末端位姿到关节轨迹

[知识库首页](../../readme.md) · [对应的 G1 硬件插件](../genie_robot_agibot_g1_hardware/README.md) · [Setup Assistant 后处理规程](MOVEIT_SETUP_ASSISTANT_POSTPROCESS.md) · [English procedure](MOVEIT_SETUP_ASSISTANT_POSTPROCESS_EN.md)

给 G1 左腕一个目标位姿，MoveIt 需要求出各关节的目标角，找到沿途可通行的路径，再决定每个轨迹点应在什么时候到达。本目录保存 A2D 在 ROS 2 Humble 下选择的插件和参数。求解器的实现随 MoveIt 提供，这里能查到每种算法用于哪个规划组、接受哪些约束。

## 各算法在规划链中的位置

| 要解决的问题 | 算法和作用 | 配置依据 |
| --- | --- | --- |
| 手臂末端要到这个位置和朝向，七个关节各转到哪里？ | 左、右臂使用 LMA（Levenberg–Marquardt）逆运动学。它从初始关节角迭代缩小末端位姿误差，得到一组候选关节角。超时可能无解，沿途是否碰撞还要交给规划器。 | [双臂的求解时限与尝试次数](config/kinematics.yaml) |
| 头部和躯干如何表达目标？ | 两组都登记了 KDL 链式逆运动学插件，用于从末端位姿迭代求关节值。项目中的头、躯干主要直接接受关节目标，这类请求可跳过位姿求解。 | [KDL 插件配置](config/kinematics.yaml) · [各组所含关节](config/A2D.srdf) |
| 起点和终点都可达，中间怎样绕过去？ | 双臂默认使用 OMPL 的 RRTConnect。它从起点与目标分别扩展采样树，尝试用通过有效性检查的局部路径连接两棵树，输出有序的关节路径。 | [双臂默认规划器](config/ompl_planning.yaml) |
| 移动过程中工具还要保持指定朝向，怎样采到有效状态？ | 对带位姿类路径约束的请求，双臂启用约束状态空间，将采样投影到满足约束的集合。每臂另选肩部两个关节作低维投影，供规划器组织探索。 | [约束空间与 projection_evaluator](config/ompl_planning.yaml) |
| 路径上每一点何时到达？ | `AddTimeOptimalParameterization` 对几何路径进行时间参数化，结合关节速度、加速度限制生成带时间的轨迹。普通请求的默认速度和加速度缩放均为 0.1。 | [请求适配器](config/ompl_planning.yaml) · [关节限位](config/joint_limits.yaml) |

OMPL 预处理器会解析约束所用的坐标系，补齐工作空间边界，并处理越界、碰撞或偏离路径约束的起始状态。排查“还没开始搜索就失败”时要先读这些参数。目录另存了 Pilz 的平移速度、加减速度和旋转速度上限。双臂的默认规划器由 OMPL 配置指定。[请求适配器顺序](config/ompl_planning.yaml) · [Pilz 笛卡尔限值](config/pilz_cartesian_limits.yaml)

## 从模型一路查到控制器

1. **模型和碰撞。** 顶层 xacro 从外部 `genie_description` 引入结构 URDF，再接上本包的控制接口。SRDF 定义左右臂、双臂组合、头、躯干与夹爪，登记末端和可跳过的碰撞对。左右第二关节收窄的方向相反，可以在限位表里读到各自的弧度范围。数值参数使用浮点字面量。[模型入口](config/A2D.urdf.xacro) · [SRDF](config/A2D.srdf) · [限位表](config/joint_limits.yaml)
2. **接口和反馈。** 控制 xacro 给出每个关节的位置命令接口、状态接口和初值，指定硬件类名 `genie_hardware/GenieArmHardware`。类名须与硬件包的插件注册逐字一致，话题参数须能被插件的 `on_init` 读到。这份 G1 快照直接指定硬件插件，原工程的接入规程另有 mock 分支示例。[控制 xacro](config/A2D.ros2_control.xacro) · [初值表](config/initial_positions.yaml) · [插件注册](../genie_robot_agibot_g1_hardware/genie_hardware.xml)
3. **轨迹分发。** MoveIt 按轨迹包含的关节选择控制器。比如 `both_arms` 的 14 关节轨迹会拆给左右两个控制器，各自通过 `FollowJointTrajectory` 接收并按时跟踪本臂目标。核对左右臂、头、躯干与左右夹爪的六个控制器时，沿下列三个文件检查同一个名字、关节列表和加载项。硬件 launch 同时按包名 `genie_moveit_config` 查找配置。[MoveIt 动作映射](config/moveit_controllers.yaml) · [500 Hz 控制器配置](config/ros2_controllers.yaml) · [控制器加载项](../genie_robot_agibot_g1_hardware/launch/hardware.launch.py)

若要理解 Setup Assistant 产物如何组装，可对照 [move_group 配置入口](launch/move_group.launch.py)、[RViz 入口](launch/moveit_rviz.launch.py)和[可视化配置](config/moveit.rviz)。本库保留这些文件供离线阅读。
