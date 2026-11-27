# G2 深度相机与 OctoMap 参数

同一条射线在两份配置里各有一个含义。感知节点决定它是否代表可信的深度观测，MoveIt 决定把沿途的格清空还是把终点记成占据。查点云为何消失或地图为何残留时，需按同一段距离对照两边。[感知节点参数](head_camera.yaml) · [MoveIt 传感器参数](sensors_3d.yaml) · [过滤算法说明](../README.md)

## 必须对齐的量

| 对照项 | 当前配置与含义 |
| --- | --- |
| 占据距离与清空距离 | 主节点 `occupy_range` 和三路 MoveIt 更新器的 `max_range` 均为 2.5 m。主节点的 `clear_distance` 为 3.0 m，清扫器的 `clip_range` 为 2.6 m，均超出占据量程，MoveIt 只沿这类远点的射线清空。清扫器代码自身的 `max_range` 默认也是 2.5 m。[主节点](../head_depth_cloud.py) · [清扫器](../octomap_sweeper.py) |
| 体素尺度 | MoveIt 的 `octomap_resolution`、主节点的 `confirm_res` 和补全间距 `shadow_step` 均为 0.03 m。帧间重复命中、连通块格数和背面补点都按这个尺度解释。[体素键与连通块](../voxel_util.py) · [背面补点](../shadow_cloud.py) |
| 过滤话题和频率 | `head_depth` 输出的 `filtered_cloud_topic` 是 `shadow_cloud` 的输入。原工程记录的 Humble 更新器要求三路均配置该字段。主点云与补全上限为 5 Hz，清扫器 `burst_rate` 和 `head_sweep.max_update_rate` 均为 10 Hz。[三路更新器](sensors_3d.yaml) |
| 机器人轮廓 | `self_mask_*` 控制深度图上的手臂轮廓。MoveIt 的 `padding_offset: 0.10`（米）控制点云自身过滤的模型外扩，避障碰撞几何由规划场景另行定义。末端额外球形遮罩半径当前为 0。[图像投影遮罩](../robot_mask.py) · [MoveIt 参数](sensors_3d.yaml) |

## 外参与坐标系

深度图尺寸为 640 × 400，焦距 `fx = fy = 309.359`，主点为 `(319.0, 202.5)`。深度值的单位为毫米。节点发布从 `head_link3` 经 `head_color_optical_frame` 到 `head_depth_optical_frame` 的静态 TF。当前 YAML 将 GDK 的头部 RGBD 外参用作深度相机在头部的位姿，因此第二段 `depth_in_color` 设为单位变换。早期叠加了 2.38 cm 横向平移，表现为定位向一侧偏约 3 cm。[标定与更正记录](head_camera.yaml) · [物体定位使用的同一外参](../../g2_object_perception/scene3d.py)

地图以 `base_link` 为参考系，转头时由 TF 将深度点换回车体坐标。底盘移动会带着地图移动，坐标系选择与清扫器对“高度”的解释都针对原地操作。[地图配置](sensors_3d.yaml) · [按高度识别悬空体素](../octomap_sweeper.py)

## 配置与源码的版本差异

这份 YAML 含 `connect_keep_points: 200`，注释意图是给足够大的悬空采样块保留表面。当前主节点的连通判断只检查贴地、画面边缘和遮挡。YAML 还给主节点和补全节点都写了 `ground_clear_z: 0.035`，这两份 Python 快照均未读取该键。源码中生效的是 `ground_max_z: 0.03`，用于判定连通块是否接地。分析地面是否写入地图或大块为何被删时，要按这一区别解释结果。[节点参数](head_camera.yaml) · [主节点连通判断](../head_depth_cloud.py) · [补全节点](../shadow_cloud.py)

[返回头部点云](../README.md) · [查看节点与话题连接](../launch/README.md)
