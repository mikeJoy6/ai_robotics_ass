# config

| 文件 | 原位置 | 内容 |
|---|---|---|
| `head_camera.yaml` | `genie_robot_agibot_g2_perception/config/` | 三个节点的参数：头部深度相机标定（内参 fx=fy=309.359 cx=319 cy=202.5；head_link3→彩色→深度外参）、深度范围/stride/频率、每一道过滤的开关与阈值（散斑、跳变、裁边、悬空块、多帧确认、小块、robot_mask）、shadow_cloud、octomap_sweeper。每个参数旁有注释和实测依据 |
| `sensors_3d.yaml` | `genie_moveit_config/config/` | MoveIt 3D 感知：octomap 在 base_link、3cm、max_range 2.5m；三个 `PointCloudOctomapUpdater` 传感器 head_depth / head_shadow / head_sweep。`MoveItConfigsBuilder` 自动读它 |

参数间的联动（改一个要同步另一个）：
- `head_camera.yaml` 的 `occupy_range` = `sensors_3d.yaml` 的 `max_range`（2.5）；`clear_distance`、sweeper 的 `clip_range` 必须 > 它。
- `confirm_res` = `octomap_resolution`（0.03）。
- 每个传感器都必须写 `filtered_cloud_topic`（Humble 必填）。
- `padding_offset` 只影响自身过滤，不影响碰撞检查。
