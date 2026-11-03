import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler , TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

def generate_launch_description():
    # 获取路径
    genie_moveit_config = get_package_share_directory('genie_moveit_config')
    xacro_path = os.path.join(genie_moveit_config, 'config', 'A2D.urdf.xacro')
    controllers_yaml = os.path.join(genie_moveit_config, 'config', 'ros2_controllers.yaml')


    sensors_yaml = os.path.join(genie_moveit_config, 'config', 'sensors.yaml')

    robot_desc = ParameterValue(Command(['xacro ', xacro_path]), value_type=str)

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

    spawn_head  = create_spawner('head_group_controller')
    spawn_waist = create_spawner('body_group_controller')

    # 夹爪的两个规划组控制器(Setup Assistant 加的 left_hand/right_hand)一直没有被 spawn 过，
    # 之前只在 ros2_controllers.yaml 里"配置了"但没人真正加载激活，MoveIt 找不到它们的
    # action server（"Action client not connected: left_hand_controller/follow_joint_trajectory"）。
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
