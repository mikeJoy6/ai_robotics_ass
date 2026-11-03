# 头部深度图 → 点云（+ 相机静态 TF），给 MoveIt PointCloudOctomapUpdater 用。hardware.launch.py 默认会 include 它。
#   ros2 launch genie_robot_agibot_g2_perception head_depth_cloud.launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory("genie_robot_agibot_g2_perception"), "config", "head_camera.yaml")
    return LaunchDescription([
        Node(package="genie_robot_agibot_g2_perception", executable="head_depth_cloud",
             name="head_depth_cloud", parameters=[params], output="screen"),
        # 背面补全：吃 MoveIt 滤掉机器人后的点云(sensors_3d.yaml head_depth 的 filtered_cloud_topic)
        Node(package="genie_robot_agibot_g2_perception", executable="shadow_cloud",
             name="shadow_cloud", parameters=[params], output="screen"),
        # octomap 里的零星格（块 <4 格）沿相机射线清掉（sensors_3d.yaml 的 head_sweep）
        Node(package="genie_robot_agibot_g2_perception", executable="octomap_sweeper",
             name="octomap_sweeper", parameters=[params], output="screen"),
    ])
