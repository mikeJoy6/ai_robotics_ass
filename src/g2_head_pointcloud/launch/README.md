# G2 头部点云的三个节点怎样配合

启动描述从上游 ROS 包 `genie_robot_agibot_g2_perception` 的共享目录读取 `config/head_camera.yaml`，把同一份 YAML 交给三个节点，各节点按自己的名称读取参数。MoveIt 的三个点云更新器由原工程的 `move_group` 加载，形成下面的连接。[启动描述](head_depth_cloud.launch.py) · [节点参数](../config/head_camera.yaml) · [MoveIt 话题配置](../config/sensors_3d.yaml)

| 节点 | 输入 | 运算及交给 MoveIt 的结果 |
| --- | --- | --- |
| [head_depth_cloud](../head_depth_cloud.py) | `/gdk/camera/head_depth` 深度图、关节反馈、TF 和 URDF | 对齐拍摄时间，反投影深度并去除手臂、散斑、悬空和不稳定点，发布 `/head_camera/depth_points`。MoveIt 的 `head_depth` 做自身过滤，滤后点云发往 `/head_camera/filtered_points`。 |
| [shadow_cloud](../shadow_cloud.py) | `/head_camera/filtered_points` 和手臂基座的 TF | 在手臂可达范围内，沿视线向表面后方补点，同一帧发布原表面与补点到 `/head_camera/shadow_points`。由 `head_shadow` 写入遮挡区的占据格。 |
| [octomap_sweeper](../octomap_sweeper.py) | `/get_planning_scene` 返回的占据地图、相机 TF、可选的 BOX 清除区域 | 找出零星、悬空或落在清除区域内的占据格。向 `/head_camera/sweep_points` 发布量程外的清空端点及沿途保护点，由 `head_sweep` 更新已有地图。 |

从深度图能得到表面点，而看不到推测的厚度时，先对照 `head_depth.filtered_cloud_topic` 和 `shadow_cloud.input_topic`，两端都应是 `/head_camera/filtered_points`。已有的小块残留则看另外一条支路，清扫器直接读规划场景，不消费背面补全的话题。[配置联动说明](../config/README.md) · [返回头部点云算法](../README.md)
