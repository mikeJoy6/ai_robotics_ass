# 2026-11-23

## 重写机器人知识库的阅读入口与算法说明

- 将根目录的 README 统一为 [readme.md](readme.md)。从 G2 找手机、G1 跟踪关节命令和捕获点推导三个问题进入相应资产，说明本库的离线用途。
- 重写 [G1 MoveIt 配置](src/genie_moveit_config/README.md)，区分逆解、约束采样、路径搜索与时间参数化。补写 [G1 硬件接口](src/genie_robot_agibot_g1_hardware/README.md)，将反馈同步、逐周期限步、夹爪单位换算与六维力诊断链接到实现。
- 重写 [G2 手臂逆运动学](src/g2_arm_ik/README.md)、[IK 对比框架](src/g2_arm_ik/ik_lab/README.md)和[算法索引](src/g2_arm_ik/ik_lab/algorithms/README.md)，交代候选解的生成、筛选、路径验收与历史测试范围。[全身关节运动](src/g2_whole_body/README.md)补全直达、中转、双向树搜索和分段计时的说明。
- 重写 [G2 绕障规划](src/g2_detour_planning/README.md)、[九种规划器](src/g2_detour_planning/planners/README.md)、[离线评测](src/g2_detour_planning/bench/README.md)与[场景工具](src/g2_detour_planning/tools/README.md)，将每种选路方法与碰撞判据、数据快照和评测口径关联起来。
- 重写 [G2 深度点云](src/g2_head_pointcloud/README.md)、[点云配置](src/g2_head_pointcloud/config/README.md)和[节点组合](src/g2_head_pointcloud/launch/README.md)。按工作树核对相机运动后的清扫行为，并标出已在配置中列出但当前源码尚未读取的参数。
- 重写 [G2 物体感知](src/g2_object_perception/README.md)与[模型工具调用](src/g2_llm_harness/README.md)。说明按手臂或相机距离选择物体的区别，以及检测结果如何变成带编号、时效与检查结果的运动目标。
