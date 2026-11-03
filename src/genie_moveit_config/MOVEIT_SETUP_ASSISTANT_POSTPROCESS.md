# TASK: 后处理 MoveIt Setup Assistant 生成的 `*_moveit_config` 包，使其接入真机 ros2_control 硬件插件

<!-- 本文件的读者是 AI agent，不是人。英文版：MOVEIT_SETUP_ASSISTANT_POSTPROCESS_EN.md（内容等价，改一份必须同步另一份）。 -->

## 元信息

| 项 | 值 |
|---|---|
| 输入 | `ros2 run moveit_setup_assistant moveit_setup_assistant` 刚生成的 `<pkg>`（目录名形如 `<robot>_moveit_config`） |
| 输出 | 同一个包，满足：真机模式 xacro 展开出真机插件 + 全部 param；默认模式仍是 mock；move_group 无参数类型错误；三处控制器名一致 |
| 环境 | ROS 2 Humble，`colcon` 工作空间 |
| 参考实现 | 本仓库 `src/genie_moveit_config/`（G1 / A2D 机器人，已完成全部步骤，冲突时以它为准） |
| 硬件插件参考 | 本仓库 `src/genie_robot_agibot_g1_hardware/`（`genie_hardware.xml`、`src/arm_control_ros.cpp`、`launch/hardware.launch.py`） |

## 占位符

| 占位符 | 含义 | 获取方式 |
|---|---|---|
| `<pkg>` | moveit_config 包名 = 目录名 | `basename` 包目录 |
| `<robot>` | URDF 机器人名，决定 `config/<robot>.*.xacro` / `<robot>.srdf` 文件名 | `ls config/*.srdf` |
| `<hw_pkg>` | 硬件插件所在包 | 找含 `<ros2_control plugin=...>` export 的 `package.xml` |
| `<plugin_src>` | 硬件插件源文件 | `<hw_pkg>/src/*.cpp` 中继承 `hardware_interface::SystemInterface` 的那个 |
| `<plugin_class>` | 插件类名字符串 | `<hw_pkg>/*.xml` 中 `<class name="...">` 的值 |

## 执行规则

1. 按 §0 → §7 顺序执行。每节末尾有 `CHECK`，不通过不得进入下一节。
2. 只改本文列出的字段；生成文件里其余内容保持原样。
3. 本文未覆盖的新机器人数值（速度、加速度、话题名）：从 `<plugin_src>` 和参考实现推导；推导不出来就停下问用户，不要编造。
4. 不得启动真机 launch（`hardware.launch.py` 等）。插件 `on_activate` 后立即向电机下发命令。真机启动只能由在场的用户执行。

---

## §0 统一包名

Setup Assistant 最后一页 "Configuration Files" 填的包名被写进 3 处，三处必须都等于目录名 `<pkg>`：

```bash
cd <pkg目录>
grep -n "<name>" package.xml          # 期望 <name><pkg></name>
grep -n "^project(" CMakeLists.txt    # 期望 project(<pkg>)
grep -n "package_name=" launch/*.py   # 期望 MoveItConfigsBuilder("<robot>", package_name="<pkg>")
```

已知坏值：`1`（用户在该页误填）。

`CHECK`：三条命令输出的名字全部等于 `<pkg>`。

注意：参考实现的包名 `genie_moveit_config` 被硬件 launch 用 `get_package_share_directory('genie_moveit_config')` 引用，不得改名。

---

## §1 `config/<robot>.ros2_control.xacro`

生成原样：

```xml
<xacro:property name="initial_positions" value="${load_yaml(initial_positions_file)['initial_positions']}"/>
<ros2_control name="${name}" type="system">
    <hardware>
        <!-- By default, set up controllers for simulation. This won't work on real hardware -->
        <plugin>mock_components/GenericSystem</plugin>
    </hardware>
```

### §1a `load_yaml` → `xacro.load_yaml`

```xml
<xacro:property name="initial_positions" value="${xacro.load_yaml(initial_positions_file)['initial_positions']}"/>
```

原因：裸 `load_yaml()` 已弃用，每次展开打印 `warning: Using load_yaml() directly is deprecated`。

### §1b `<plugin>` 换成 `<plugin_class>`

- 字符串必须与 `<hw_pkg>` 的插件 xml 中 `<class name="...">` 逐字一致。pluginlib 按类名查找，与包名无关。
- 已知值：

| 机器人 | `<plugin_class>` | `<plugin_src>` |
|---|---|---|
| G1 (A2D) | `genie_hardware/GenieArmHardware` | `src/genie_robot_agibot_g1_hardware/src/arm_control_ros.cpp`（本仓库） |
| G2 | `genie_hardware/GenieG2ArmHardware` | `agi@g2_robot/src/genie_robot_agibot_g2_hardware/src/arm_control_ros.cpp`（不在本仓库） |

- 新机器人必须使用新类名，禁止复用 `GenieArmHardware`。原因：同时 source 两个工作空间时类名冲突。

### §1c 补齐 `<param>`

插件 `on_init()` 用 `info_.hardware_parameters.at("<key>")` 读参数；缺任一 key 即 FATAL（日志：`URDF ros2_control 标签中缺少必要参数!`）。
所需 param 集合 = 下面命令的输出，一个不能少：

```bash
grep -o 'hardware_parameters.at("[^"]*")\|read_bool_param("[^"]*"' <plugin_src> | sort -u
```

已知集合：

- G1：`left_state_topic` `right_state_topic` `neck_state_topic` `body_state_topic` `left_ee_state_topic` `right_ee_state_topic` `command_topic` `head_command_topic` `body_command_topic` `left_ee_command_topic` `right_ee_command_topic` `control_head`
- G2：`state_topic=/gdk/joint_state` `status_topic=/gdk/motion_control_status` `command_topic=/gdk/joint_control` `control_head` `control_body`

开关类 param（`control_head`、`control_body`）的值写成 `$(arg <name>)`，在 §2 声明。

### §1d 保留 mock 分支（新机器人必须做；G1 参考实现没做，写死了真机插件）

```xml
<hardware>
    <xacro:if value="$(arg use_real_hardware)">
        <plugin><plugin_class></plugin>
        <param name="state_topic">/gdk/joint_state</param>
        <!-- §1c 的其余 param -->
        <param name="control_head">$(arg control_head)</param>
    </xacro:if>
    <xacro:unless value="$(arg use_real_hardware)">
        <plugin>mock_components/GenericSystem</plugin>
    </xacro:unless>
</hardware>
```

目的：`demo.launch.py` 仍可纯仿真运行。

### §1e `<joint>` 块

不修改内容（command_interface position；state_interface position、velocity；initial_value）。
但必须核对三个集合相等：本文件 `<joint name>` 集合 = 硬件反馈中的关节集合 = SRDF 各规划组关节并集。
已知遗漏：G1 夹爪 `left_narrow2_joint`、`right_narrow2_joint` 需手动补入。

`CHECK`（§7 第 1、2 条命令）：真机模式 `<hardware>` 含 `<plugin_class>` 与 §1c 全部 param；默认模式为 mock；无 deprecated 警告。

---

## §2 `config/<robot>.urdf.xacro`

### §2a 声明 §1 用到的每个 `$(arg ...)`

生成原样只声明 `initial_positions_file`。未声明的 arg 会导致展开报错 `Undefined substitution argument`。

```xml
<xacro:arg name="initial_positions_file" default="initial_positions.yaml" />
<xacro:arg name="use_real_hardware" default="false" />   <!-- 仅当做了 §1d -->
<xacro:arg name="control_head" default="true" />
<xacro:arg name="control_body" default="true" />          <!-- 仅当插件读取 control_body（G2） -->
```

### §2b 宏调用的 `name`

`FakeSystem` → 有意义的名字。G1 用 `GenieHardware`：

```xml
<xacro:A2D_ros2_control name="GenieHardware" initial_positions_file="$(arg initial_positions_file)"/>
```

### §2c 硬件 launch 传参约定

```python
Command(['xacro ', xacro_path, ' use_real_hardware:=true', ' control_head:=', control_head, ' initial_positions_file:=', abs_path])
```

`initial_positions_file` 必须传绝对路径。相对路径会被拼接到 xacro 文件所在目录。

`CHECK`：`xacro <robot>.urdf.xacro`（不带任何参数）展开成功，无 `Undefined substitution argument`。

---

## §3 `config/joint_limits.yaml`

### §3a 所有数值必须是浮点字面量

生成器写出整数（如 `max_acceleration: 0`、`max_velocity: 3`）。MoveIt 以 double 类型声明这些参数，整数导致：

```
parameter 'robot_description_planning.joint_limits.xxx.max_acceleration' has invalid type: expected [double] got [integer]
```

结果：move_group 启动失败，或该关节限位被忽略。

规则：`max_velocity` `max_acceleration` `min_position` `max_position` 一律带小数点（`0`→`0.0`，`5`→`5.0`）。

### §3b 启用加速度限制

生成原样 `has_acceleration_limits: false` + `max_acceleration: 0`。TOTG 时间参数化需要加速度上限，否则轨迹过激或警告 `No acceleration limit was defined`。每个关节改为：

```yaml
  <joint>:
    has_velocity_limits: true
    max_velocity: <表中值>
    has_acceleration_limits: true
    max_acceleration: <表中值>
```

取值（G1 参考；新机器人按关节类型套用，无法判断时取表中较小值）：

| 关节类型 | max_velocity | max_acceleration |
|---|---|---|
| 手臂 7 关节 | 5.0 | 1.0 |
| 头部 | 1.0 | 1.0 |
| 腰 / 升降 | 0.10 | 0.10 |
| 夹爪 | 10.0 | 2.0 |

`default_velocity_scaling_factor` 与 `default_acceleration_scaling_factor` 保持 `0.1`。

### §3c 位置限位（仅当用户要求收窄时）

`has_position_limits: true` + `min_position` / `max_position`（单位 rad）覆盖 URDF `<limit>`。修改时必须同步修改 URDF，两处保持一致。G1 先例：Joint2 收窄到 ±100°。

`CHECK`（应无输出）：

```bash
grep -nE "(max|min)_(velocity|acceleration|position): -?[0-9]+$" config/joint_limits.yaml
```

---

## §4 `config/moveit_controllers.yaml`

每个控制器在 `type` 与 `joints` 之间加两行：

```yaml
  left_arm_group_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory   # action 名 = <controller>/follow_joint_trajectory
    default: True                        # 该规划组默认用此控制器执行
    joints:
      - ...
```

约束：

- `controller_names` 列表 = 本文件定义的控制器 = `ros2_controllers.yaml` 中的控制器，名字逐字一致。
- 每个控制器 `joints` = SRDF 对应规划组的关节，顺序一致。
- 机器人有夹爪且硬件反馈含夹爪 → 必须添加 `left_hand_controller`、`right_hand_controller`（G1 有；G2 GDK 反馈无夹爪，不加）。
  遗漏症状：`Action client not connected: left_hand_controller/follow_joint_trajectory`。

`CHECK`：§7 第 4 条命令两边输出相同。

---

## §5 `config/ros2_controllers.yaml`

### §5a `update_rate`

```yaml
controller_manager:
  ros__parameters:
    update_rate: 500  # Hz，生成原样为 100
```

约束：插件 `write()` 每周期最多移动 `STEP_SIZE = 0.01 rad`（见 `<plugin_src>`），故关节最高速度 = `0.01 × update_rate` rad/s。

| 机器人 | 反馈频率 | update_rate | 最高速度 |
|---|---|---|---|
| G1 | ≥500 Hz | 500 | 5 rad/s |
| G2 | `/gdk/joint_state` 100 Hz | 100 | 1 rad/s |

不得让 update_rate 高于硬件反馈频率，除非用户明确要求；如用户要求，告知其须在真机小幅度测试。

### §5b 新增 `joint_state_broadcaster` 参数段（生成原样没有）

```yaml
joint_state_broadcaster:
  ros__parameters:
    publish_rate: 500    # 等于 update_rate
```

### §5c 每个 JointTrajectoryController 加 3 个字段

```yaml
left_arm_group_controller:
  ros__parameters:
    joints: [...]
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    state_publish_rate: 100.0          # 新增
    action_monitor_rate: 20.0          # 新增
    allow_partial_joints_goal: false   # 新增
```

### §5d 夹爪控制器（与 §4 同条件）

1. `controller_manager.ros__parameters` 下声明 `left_hand_controller`、`right_hand_controller`，type `joint_trajectory_controller/JointTrajectoryController`。
2. 添加对应 `ros__parameters` 段（格式同 §5c）。
3. 硬件 launch 中为它们创建 spawner。G1 先例：曾配置但未 spawn，MoveIt 找不到 action。

`CHECK`：§7 第 4 条；硬件 launch 中 spawner 名单 = `ros2_controllers.yaml` 控制器名单。

---

## §6 其它文件（按条件执行）

| 文件 | 条件 | 操作 |
|---|---|---|
| `kinematics.yaml` | 总是 | 生成原样 `kinematics_solver_timeout: 0.0050000000000000001` 过短。手臂组：`lma_kinematics_plugin/LMAKinematicsPlugin`、timeout `0.08`、attempts `10`；头/腰组：KDL、timeout `0.05`。删除 SRDF 中不存在的组（核对：`grep "group name" config/*.srdf`；G2 先例：多余的 `left_group`） |
| `ompl_planning.yaml` | 仅当需要朝向路径约束（如斜抓） | 生成器不产出此文件。从参考实现复制整份；全局块 `planning_plugin`、`request_adapters` 原样保留 |
| `initial_positions.yaml` | 不改 | 只被 mock 使用；真机插件 `on_activate` 用实际反馈覆盖 |
| `launch/*.py` | 不改 | 除 §0 的 `package_name`。真机启动使用 `<hw_pkg>/launch/hardware.launch.py` |

---

## §7 最终验证（全部通过才算完成）

```bash
cd <工作空间>
source /opt/ros/humble/setup.bash
source ~/.cache/agibot/app/gdk/examples/ros/install/local_setup.bash   # 仅 G2：编译需要 gdk_msgs
colcon build && source install/setup.bash

C=$PWD/install/<pkg>/share/<pkg>/config
# 1) 真机模式：<hardware> 内为 <plugin_class> + §1c 全部 param；stderr 无 deprecated
xacro $C/<robot>.urdf.xacro use_real_hardware:=true initial_positions_file:=$C/initial_positions.yaml | grep -A20 "<hardware>"
# 2) 默认模式：mock_components/GenericSystem（仅当做了 §1d）
xacro $C/<robot>.urdf.xacro initial_positions_file:=$C/initial_positions.yaml | grep -A2 "<hardware>"
# 3) 应无输出
grep -nE "(max|min)_(velocity|acceleration|position): -?[0-9]+$" $C/joint_limits.yaml
# 4) 两条输出必须相同
grep -E "^\s+- .*_controller$" $C/moveit_controllers.yaml | sed 's/^\s*- //' | sort
grep -E "^[a-z_]+_controller:$" $C/ros2_controllers.yaml | sed 's/:$//' | sort
# 5) launch 参数可解析（只解析，不启动）
ros2 launch <hw_pkg> hardware.launch.py --show-args
```

完成后向用户报告每条 CHECK 的结果，并转告真机启动步骤（由用户执行，不由你执行）：
1. 机器人旁有人看护时启动 `ros2 launch <hw_pkg> hardware.launch.py`。
2. 日志 `Activation feedback ready? ...` 各项全部为 1。
3. 用 MoveIt 小幅度规划试动。

---

## 速查表

| 文件 | 改动 |
|---|---|
| `package.xml` / `CMakeLists.txt` / `launch/*.py` | 包名 = 目录名（已知坏值 `1`） |
| `<robot>.ros2_control.xacro` | `load_yaml`→`xacro.load_yaml`；`<plugin>`→`<plugin_class>`；补齐全部 `<param>`；mock 分支 |
| `<robot>.urdf.xacro` | 声明 `use_real_hardware` / `control_head` 等 arg；宏 `name`→`GenieHardware` |
| `joint_limits.yaml` | 全部数值带小数点；`has_acceleration_limits: true` + `max_acceleration` |
| `moveit_controllers.yaml` | 每个控制器加 `action_ns: follow_joint_trajectory`、`default: True`；夹爪控制器 |
| `ros2_controllers.yaml` | `update_rate`；`joint_state_broadcaster.publish_rate`；每个 JTC 加 `state_publish_rate` / `action_monitor_rate` / `allow_partial_joints_goal`；夹爪控制器 |
| `kinematics.yaml` | timeout 加大；手臂用 LMA；删多余组 |
