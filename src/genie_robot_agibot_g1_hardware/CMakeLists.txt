cmake_minimum_required(VERSION 3.8)
project(genie_robot_agibot_g1_hardware)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. 寻找包（仅 arm_control_ros 插件所需）
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(genie_msgs REQUIRED)      # 机器人专属 SDK 驱动消息

# 2. 编译硬件接口插件（控制 L0 —— SystemInterface 插件，唯一产物）
add_library(arm_control_ros SHARED src/arm_control_ros.cpp)
ament_target_dependencies(arm_control_ros
  rclcpp
  std_msgs
  sensor_msgs
  genie_msgs
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

# 3. 向 ros2_control 导出插件描述（controller_manager 据此加载 .so）
pluginlib_export_plugin_description_file(hardware_interface genie_hardware.xml)

# 4. 安装
#   插件 .so 装到 lib 根目录（pluginlib 推荐）
install(TARGETS arm_control_ros
  DESTINATION lib)
#   插件描述 + launch
install(FILES genie_hardware.xml
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
#   诊断小工具（python 脚本，装到 lib/<pkg> 才能被 ros2 run 找到）
#   本包是 ament_cmake，没有 setup.py 的 console_scripts，用 install(PROGRAMS) 直接装可执行文件；
#   调用时带 .py 后缀：ros2 run genie_robot_agibot_g1_hardware force_monitor.py
install(PROGRAMS scripts/force_monitor.py
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rclcpp genie_msgs hardware_interface pluginlib)
ament_package()
