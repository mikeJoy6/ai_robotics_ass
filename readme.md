# ai_assistant · 机器人开发知识库

“让 G2 的右手停在手机上方。”要做到这一点，先得从画面里认出手机，算出它在机器人坐标系里的位置。手腕以什么姿态能到？肘部在途中会不会碰到桌面？这个仓库把每个问题的解法、配置和判断依据放在一起，供开发者和 AI 离线查阅。这里保存的是智元 G1、G2 的数字资产，阅读和维护都无需编译或连接机器人。

## 跟着一个 G2 任务读

**先找到手机。** 按名称查找时，SAM 3 给出图像掩码，结合深度和相机标定，可以算出手机在基座坐标系（`base_link`）里的位置。画面里有多部手机时，按到相机的距离编号。换成“离右手最近的红色积木”，还要从同色候选中按右手末端距离选择。两种“最近”用的是不同参照物。点云模块则负责分辨哪些空间被障碍物占据，哪些深度点来自机器人自身或传感器噪声。[物体定位与转头对准](src/g2_object_perception/README.md) · [深度点云与占据地图](src/g2_head_pointcloud/README.md)

**再决定手怎么过去。** 已知末端目标，逆运动学（IK）可以求出一组关节角。规划整段动作时，还得检查沿途的构型和碰撞。同一个终点可以先试笛卡尔直线，受阻时尝试中转点或搜索绕行。若任务改成“恢复录好的姿势”，输入就变成了腰、头或双臂的关节目标。[手臂逆运动学](src/g2_arm_ik/README.md) · [手臂绕障与轨迹计时](src/g2_detour_planning/README.md) · [全身关节组运动](src/g2_whole_body/README.md)

**最后看一句话如何调用这些能力。** 模型可以用 `phone_1` 指代相机找到的目标。程序负责保存坐标、计算悬停点、审查规划结果，并区分“已有方案”和“已经执行”。想查某项能力的实测记录或尚未接入的算法，可以从能力地图继续读。[模型工具与调用流程](src/g2_llm_harness/README.md) · [G2 能力地图](src/g2_llm_harness/TOOLS_MAP.md)

## 在 G1 上追踪一条关节命令

G1（模型名 A2D）保留了 ROS 2 Humble 下相互配套的两份参考包。要查“这个手腕姿态能否解出”，从 MoveIt 的规划组和运动学插件读起。要查“轨迹目标如何变成底层命令”，顺着控制器配置进入硬件插件，可以看到它怎样把第一条命令对齐到反馈、每周期最多改变多少，以及夹爪开度怎样换算。[MoveIt 配置与规划算法](src/genie_moveit_config/README.md) · [ros2_control 硬件接口](src/genie_robot_agibot_g1_hardware/README.md)

移植配置时，Setup Assistant 的生成结果只是起点。后处理规程逐项说明如何核对插件类名、关节限位、控制器和硬件参数，并标出 G1 样例与通用步骤的差异。[中文规程](src/genie_moveit_config/MOVEIT_SETUP_ASSISTANT_POSTPROCESS.md) · [English procedure](src/genie_moveit_config/MOVEIT_SETUP_ASSISTANT_POSTPROCESS_EN.md)

## 读推导，也保存推导

平衡控制笔记从线性倒立摆推到捕获点（capture point），解释为什么质心位置相同，速度不同，下一步的落脚点就会不同。公式排版指南用这份笔记作正例，也保留了一份在 VS Code 中无法正确显示公式的教案作对照。[捕获点与落脚推导](reference/CapturePointSteppingDesign.md) · [欧拉角教案与定界符反例](reference/第一课教案_欧拉角的导数为什么不是角速度.md) · [中文排版指南](reference/MARKDOWN_MATH_GUIDE.md) · [English guide](reference/MARKDOWN_MATH_GUIDE_EN.md)

## 怎样使用这些资产

G2 各目录是从原工程选录的源码快照，保留原来的导入路径和外部依赖。包内 README 说明每种算法接受什么输入、产出什么，以及哪些结果来自特定场景。维护规程时，中英文配对更新，参数追溯到源码或配置，步骤附可核对的 `CHECK` 和占位符说明，并从对应主题链接新规程。含公式的文档用[公式检查器](reference/check_md_math.py)核对。
