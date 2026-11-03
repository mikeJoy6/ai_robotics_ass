"""hardware_obstacle.launch.py —— hardware.launch.py 原样 + 在机器人前方放一个虚拟碰撞体（STL），测试避障用。

hardware.launch.py 不改；本文件 include 它（参数照传），再用 add_collision_mesh.py 把 reference/zs_liaokaung.stl
加进 MoveIt planning scene。位置用下面 obstacle_* 参数调（base_link 系：x 前 y 左 z 上，单位 m / rad）：
  ros2 launch genie_robot_agibot_g2_hardware hardware_obstacle.launch.py obstacle_xyz:="[0.8,0.0,-0.06]" obstacle_rpy:="[0,0,1.57]"
默认：STL 原点（它的底面中心偏后，网格 x∈[-0.32,1.02] y∈[-0.26,0.26] z∈[0,1.61]）放在 x=0.8 → 近端约在前方 0.48m；
z=-0.06 是实测地面高度（base_link 下 6cm）。
改位置不用重启：同 id 再加一次就是替换——
  ros2 run genie_robot_agibot_g2_hardware add_collision_mesh.py --ros-args -p mesh:=<stl> -p xyz:="'[0.9,0,-0.06]'" -p rpy:="'[0,0,0]'"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

SHARE = get_package_share_directory('genie_robot_agibot_g2_hardware')
# reference/ 有 COLCON_IGNORE、不安装：按标准工作空间布局 <ws>/install/<pkg>/share/<pkg> 往回找 <ws>/reference
DEFAULT_MESH = os.path.normpath(os.path.join(SHARE, '..', '..', '..', '..', 'reference', 'zs_liaokaung.stl'))


def generate_launch_description():
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(SHARE, 'launch', 'hardware.launch.py'))
    )
    add_mesh = Node(
        package='genie_robot_agibot_g2_hardware',
        executable='add_collision_mesh.py',
        output='screen',
        parameters=[{
            'mesh': LaunchConfiguration('obstacle_mesh'),
            'frame': 'base_link',
            'xyz': ParameterValue(LaunchConfiguration('obstacle_xyz'), value_type=str),
            'rpy': ParameterValue(LaunchConfiguration('obstacle_rpy'), value_type=str),
            'object_id': 'zs_liaokaung',
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument('obstacle_mesh', default_value=DEFAULT_MESH),
        DeclareLaunchArgument('obstacle_xyz', default_value='[0.8, 0.0, -0.06]',
                              description='STL 原点在 base_link 里的位置 [x,y,z] m'),
        DeclareLaunchArgument('obstacle_rpy', default_value='[0.0, 0.0, 0.0]',
                              description='STL 姿态 [roll,pitch,yaw] rad'),
        hardware,
        add_mesh,
    ])
