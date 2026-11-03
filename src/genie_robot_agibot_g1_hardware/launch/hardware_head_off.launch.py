import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler , TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

# hardware.launch.py 的"头部开关版"——唯一区别：xacro 传 control_head:=false，让 arm_control_ros
#   不再往 /wbc/head_command 发头命令(仍读头部状态)，把脖子让给外部 PID 伺服(grasp_teach 的 follow)。
# 其余(控制器、move_group、rviz)与原 launch 逐行相同。原 hardware.launch.py 保持不动、照常可用。
# 注意：用本 launch 时 head_l/head_m/head_r(走 MoveIt)不再真的动头——已知、可接受(见 DEVLOG)。

def generate_launch_description():
    # 获取路径
    genie_moveit_config = get_package_share_directory('genie_moveit_config')
    xacro_path = os.path.join(genie_moveit_config, 'config', 'A2D.urdf.xacro')
    controllers_yaml = os.path.join(genie_moveit_config, 'config', 'ros2_controllers.yaml')


    sensors_yaml = os.path.join(genie_moveit_config, 'config', 'sensors.yaml')

    # ★与原 launch 唯一的区别：给 xacro 传 control_head:=false★
    robot_desc = ParameterValue(
        Command(['xacro ', xacro_path, ' control_head:=false']), value_type=str)

    # 1. 启动硬件驱动 (ros2_control_node)
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_desc}, controllers_yaml],
        output='screen',
    )

    # 2. 启动 robot_state_publisher (替代你手动运行的那个)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
    )

    # 3. 自动加载控制器 (通过 spawner)
    def create_spawner(name):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '--controller-manager', '/controller_manager'],
        )

    # 确保控制器在 ros2_control_node 启动后再加载
    spawn_jsb = create_spawner('joint_state_broadcaster')
    spawn_left = create_spawner('left_arm_group_controller')
    spawn_right = create_spawner('right_arm_group_controller')

    # head_group_controller 仍照常 spawn：让 head_l/m/r 不至于报"action server not connected"，
    #   只是 write() 被 control_head=false 拦住不发头命令 → 头不动但不报错(保留的 bug，见 DEVLOG)。
    spawn_head  = create_spawner('head_group_controller')
    spawn_waist = create_spawner('body_group_controller')

    spawn_left_hand  = create_spawner('left_hand_controller')
    spawn_right_hand = create_spawner('right_hand_controller')


    # 4. 启动 MoveIt (move_group)
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(genie_moveit_config, 'launch', 'move_group.launch.py')
        )
    )

    # 5. 启动 RViz (可选，如果你想一键全开)
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(genie_moveit_config, 'launch', 'moveit_rviz.launch.py')
        )
    )

    scene_pub_node = Node(
        package='moveit_ros_planning_interface',
        executable='moveit_publish_scene_from_text',
        arguments=[os.path.join(genie_moveit_config, 'config', 'table.scene')],
        output='screen',
    )

    delayed_scene_pub = TimerAction(
        period=5.0, # 给机器人启动和发布TF留出 5 秒缓冲时间
        actions=[scene_pub_node]
    )


    return LaunchDescription([
        control_node,
        rsp_node,
        RegisterEventHandler(OnProcessStart(target_action=control_node, on_start=[spawn_jsb])),
        RegisterEventHandler(OnProcessStart(target_action=spawn_jsb, on_start=[spawn_left])),
        RegisterEventHandler(OnProcessStart(target_action=spawn_left, on_start=[spawn_right])),

        RegisterEventHandler(OnProcessStart(target_action=spawn_right, on_start=[spawn_head])),
        RegisterEventHandler(OnProcessStart(target_action=spawn_head, on_start=[spawn_waist])),
        RegisterEventHandler(OnProcessStart(target_action=spawn_waist, on_start=[spawn_left_hand])),
        RegisterEventHandler(OnProcessStart(target_action=spawn_left_hand, on_start=[spawn_right_hand])),

        move_group_launch,
        rviz_launch,
        # delayed_scene_pub
    ])
