# launch

`head_depth_cloud.launch.py`：用 `config/head_camera.yaml` 起三个节点——`head_depth_cloud`（主节点）、`shadow_cloud`（背面补全）、`octomap_sweeper`（零星格清扫）。

真机上由 `genie_robot_agibot_g2_hardware/launch/hardware.launch.py` 默认 include（`use_head_cloud:=false` 关），也可单独起：
`ros2 launch genie_robot_agibot_g2_perception head_depth_cloud.launch.py`（需要 move_group 在跑、`sensors_3d.yaml` 已被 MoveIt 加载）。
