# G1 硬件接口 · 一条关节命令的往返

[知识库首页](../../readme.md) · [配套 MoveIt 配置](../genie_moveit_config/README.md) · [插件实现](src/arm_control_ros.cpp)

`GenieArmHardware` 连接 ros2_control 的关节接口和 G1 的底层话题。读方向是 HAL 反馈 → 按 URDF 关节名存放的位置，写方向是控制器目标 → 限制每周期变化量 → WBC 命令。硬件接口的职责始于关节目标，末端逆运动学与路径搜索由上游 MoveIt 处理。[插件注册](genie_hardware.xml) · [参数与话题映射](../genie_moveit_config/config/A2D.ros2_control.xacro)

## 反馈怎样变成下一条命令

| 处理方法 | 输入、规则和结果 |
| --- | --- |
| 关节映射与反馈快照 | 双臂按电机数组顺序和左右侧映射成 `Joint1_l` 等名字。头、腰按反馈名称兼容别名。回调把有限位置值写入受锁保护的缓冲，`read()` 每周期复制一份给控制器。本实现导出速度状态接口，但速度数组只有初始化值，不能当作测量结果。[手臂映射](src/arm_control_ros.cpp#L168) · [头部映射](src/arm_control_ros.cpp#L244) · [腰部映射](src/arm_control_ros.cpp#L296) · [状态读取](src/arm_control_ros.cpp#L514) |
| 启动与迟到反馈同步 | 激活时先请求手臂使能，再有限次等待反馈，并把当前缓冲位置复制到命令和上次输出，减少从默认零位跳变。头部需同时收到 yaw 和 pitch，腰部需同时收到 pitch 和 lift。两侧夹爪共用一个开关，必须两侧都有有效反馈才开放夹爪命令。双臂的就绪标记只表明各侧收过有限反馈，超时后激活仍可返回成功。[激活与同步](src/arm_control_ros.cpp#L430) |
| 逐周期限步 | `write()` 用控制器目标减去上次发出的值，再把差值限制在 ±0.01 个关节单位内逐步追上。按配置的 500 Hz，对旋转关节等效的**指令**上限是 5 rad/s。实际速度还取决于底层响应。[限步与分组](src/arm_control_ros.cpp#L544) · [控制频率](../genie_moveit_config/config/ros2_controllers.yaml) |
| 夹爪的单位换算与变化检测 | HAL 的 0° 表示全开、120° 表示闭合，线性映射成 `narrow2_joint` 的 −1 到 0 rad。反向写入时换算到 0 至 1 的开合分数并截断到有效范围。相邻两次平滑指令的差超过 0.0001 才转发，避免空闲控制器持续占用夹爪话题。[反馈换算](src/arm_control_ros.cpp#L345) · [发送条件](src/arm_control_ros.cpp#L580) |

核对腰部命令的数组次序时，以实际填入消息的语句为准。这份实现先追加 `joint_body_pitch`，再追加 `joint_lift_body`，旁边的历史注释描述了相反顺序。[腰部消息打包](src/arm_control_ros.cpp#L593)

## 对照启动配置与诊断工具

两份 launch 保存原工程的接线方式。常规版加载控制器、状态发布器、move_group 和 RViz。另一版把 `control_head` 传成 `false`，保留头部状态读取与控制器加载，把头部命令话题留给外部控制器。阅读这两份 launch 即可核对差别。本仓库只作离线查阅。[常规配置](launch/hardware.launch.py) · [头部交由外部控制的配置](launch/hardware_head_off.launch.py)

六维力监视器提供一个读取反馈的独立例子。它把三路力和三路力矩分别除以 200 N 和 7 N·m，显示各轴占量程的百分比，并按帧更新绝对峰值。合力和合力矩分别取三个分量的欧氏范数。数字列按真实量程显示，条形图默认在量程的 25% 处画满，便于观察小载荷。[六维力监视器](scripts/force_monitor.py)
