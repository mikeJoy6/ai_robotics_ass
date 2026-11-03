#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "genie_msgs/msg/arm_state.hpp"
#include "genie_msgs/srv/arm_control.hpp"


#include "genie_msgs/msg/head_state.hpp"
#include "genie_msgs/msg/waist_state.hpp"
#include "genie_msgs/msg/end_state.hpp"   // 夹爪反馈：/hal/{left,right}_ee_data


#include "pluginlib/class_list_macros.hpp"
#include <map>
#include <thread>
#include <algorithm>
#include <mutex>  // 解决多线程同步数据竞争
#include <limits> // std::numeric_limits（夹爪命令"是否在变"判据的 NaN 哨兵值）


namespace genie_hardware
{

class GenieArmHardware : public hardware_interface::SystemInterface
{
public:
  static constexpr double STEP_SIZE = 0.01; // 步进值：手动平滑控制的单步增量
  
  // 初始化函数：在 ros2_control 加载插件时调用
  // 此处传入修改后的urdf或者xacro
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    // 检查父类初始化：确保 URDF 解析基础无误
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // 创建内部 ROS 节点 genie_hw_interface_node
    // make_shared创建一个Node对象
    node_ = std::make_shared<rclcpp::Node>("genie_hw_interface_node");
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    try {
      // 数据来源：从 URDF 文件的 <ros2_control> -> <hardware> -> <param> 标签中读取话题名称
      // info_.hardware_parameters是键值对的读取容器，专门加载<param name=A> B </param>
      // .at强行读取搜索，读不到就报错
      // 此处用于加载ros2_control的控制接口地址名称

      // 在URDF中，常见<param name="command_topic">/wbc/arm_command</param> 
      // 其中command_topic为键，/wbc/arm_command 为值
      // 此时 cmd_topic 被传入 /wbc/arm_command
      std::string cmd_topic = info_.hardware_parameters.at("command_topic");
      std::string head_cmd_topic = info_.hardware_parameters.at("head_command_topic");
      std::string body_cmd_topic = info_.hardware_parameters.at("body_command_topic");

      
      // 读取状态的topic
      std::string left_topic = info_.hardware_parameters.at("left_state_topic");
      std::string right_topic = info_.hardware_parameters.at("right_state_topic");
      std::string neck_state_topic = info_.hardware_parameters.at("neck_state_topic");
      std::string body_state_topic = info_.hardware_parameters.at("body_state_topic");

      // 夹爪话题：反馈一直接着；命令改成"只在 JTC 真正执行轨迹(值持续变化)时才转发"，
      //   空闲悬停(值不变)时不发——这样不跟 setGripper()/GripperController 抢话题，
      //   见 DEVLOG「夹爪控制」。
      std::string left_ee_state_topic = info_.hardware_parameters.at("left_ee_state_topic");
      std::string right_ee_state_topic = info_.hardware_parameters.at("right_ee_state_topic");
      std::string left_ee_cmd_topic = info_.hardware_parameters.at("left_ee_command_topic");
      std::string right_ee_cmd_topic = info_.hardware_parameters.at("right_ee_command_topic");

      // 数据去向：发布平滑后的指令给 WBC 控制接口
      // create_publisher<消息类型>(话题名,Qos)
      pub_arm_command_ = node_->create_publisher<sensor_msgs::msg::JointState>(cmd_topic, qos);
      pub_head_command_ = node_->create_publisher<sensor_msgs::msg::JointState>(head_cmd_topic, qos);
      pub_waist_command_ = node_->create_publisher<sensor_msgs::msg::JointState>(body_cmd_topic, qos);
      // 夹爪命令：QoS 跟 GripperController 保持一致——RELIABLE + depth=1，避免积压导致的
      //   "10秒延迟"坑（这个坑之前在 gripper_primitives.cpp 踩过，见 DEVLOG）。
      pub_left_ee_command_ = node_->create_publisher<sensor_msgs::msg::JointState>(
          left_ee_cmd_topic, rclcpp::QoS(1).reliable());
      pub_right_ee_command_ = node_->create_publisher<sensor_msgs::msg::JointState>(
          right_ee_cmd_topic, rclcpp::QoS(1).reliable());

      // 绑定回调：处理左右臂的底层数据
      // 先看 const genie_msgs::msg::ArmState::SharedPtr msg，此处加载 genie_msgs 包中的 arm_state
      // 对 arm_state 使用智能指针 ::SharedPtr，创建并传入参数msg，格式为指针地址
      // 目的是将电机的真实位置打包成 ArmState 消息，发往 /hal/left_arm_data 话题，此时传给变量msg
      // 智能指针作用是，每次left_cb接收了一次消息，都会覆盖并打包为新的 msg
      // 对函数体，执行内部函数process_arm_data
      auto left_cb = [this](const genie_msgs::msg::ArmState::SharedPtr msg) { this->process_arm_data(msg, "_l"); };
      auto right_cb = [this](const genie_msgs::msg::ArmState::SharedPtr msg) { this->process_arm_data(msg, "_r"); };
      auto neck_cb = [this](const genie_msgs::msg::HeadState::SharedPtr msg) { this->process_head_data(msg); };
      auto waist_cb = [this](const genie_msgs::msg::WaistState::SharedPtr msg) { this->process_waist_data(msg); };
      auto left_ee_cb = [this](const genie_msgs::msg::EndState::SharedPtr msg) { this->process_gripper_data(msg, true); };
      auto right_ee_cb = [this](const genie_msgs::msg::EndState::SharedPtr msg) { this->process_gripper_data(msg, false); };

      // 监听 subscription
      // 订阅来自 HAL 层的真实硬件反馈话题
      // sub_left_ 代表着一种权柄，本文不做调用
      // 目的是监听 left_topic，把 genie_msgs/msg/arm_state 里的信息传入 left_cb，
      // genie_msgs/msg/arm_state 中包含14个电机的信息
      // 电机信息传给left_topic后，执行内部函数process_arm_data
      sub_left_ = node_->create_subscription<genie_msgs::msg::ArmState>(left_topic, qos, left_cb);
      sub_right_ = node_->create_subscription<genie_msgs::msg::ArmState>(right_topic, qos, right_cb);
      sub_neck_ = node_->create_subscription<genie_msgs::msg::HeadState>(neck_state_topic, qos, neck_cb);
      sub_waist_ = node_->create_subscription<genie_msgs::msg::WaistState>(body_state_topic, qos, waist_cb);
      // 夹爪反馈是传感数据、best_effort（跟 GripperController 的订阅 QoS 保持一致——见
      //   gripper_primitives.cpp 注释：丢几帧无所谓，subscriber 要求 reliable 反而可能因
      //   QoS 不兼容收不到数据，之前调头/腰话题就踩过一次这个坑）。
      sub_left_ee_ = node_->create_subscription<genie_msgs::msg::EndState>(
          left_ee_state_topic, rclcpp::QoS(10).best_effort(), left_ee_cb);
      sub_right_ee_ = node_->create_subscription<genie_msgs::msg::EndState>(
          right_ee_state_topic, rclcpp::QoS(10).best_effort(), right_ee_cb);


    } catch (const std::out_of_range& e) {
      RCLCPP_FATAL(node_->get_logger(), "URDF ros2_control 标签中缺少必要参数!");
      return hardware_interface::CallbackReturn::ERROR;
    }

    // 头部开关(control_head)：默认 true=照常控头。false=write() 不再发头命令(仍读头部状态、仍导出
    //   状态接口)，把 /wbc/head_command 让给外部 PID 伺服(follow)。用 count 而非 .at()：老 URDF 没这个
    //   参数时不报错、按 true 处理，保证"对现有 launch 无任何影响"。
    control_head_ = true;
    if (info_.hardware_parameters.count("control_head")) {
      const std::string v = info_.hardware_parameters.at("control_head");
      control_head_ = !(v == "false" || v == "False" || v == "0");
    }
    RCLCPP_INFO(node_->get_logger(), "control_head = %s",
                control_head_ ? "true（MoveIt/ros2_control 控头）"
                              : "false（硬件层不发头命令，脖子交给外部 PID）");

    // 创建客户端
    // 向底层 HAL 服务请求使能/释放机械臂
    left_client_ = node_->create_client<genie_msgs::srv::ArmControl>("/hal/left_arm_control");
    right_client_ = node_->create_client<genie_msgs::srv::ArmControl>("/hal/right_arm_control");

    // 内部存储初始化：根据 URDF 定义的关节数量分配内存
    // info_.joints.size()为URDF中定义的关节总数
    // .resize() 是 std::vector 的方法，提前分好位置，防止程序运行时临时开空间导致变慢。
    hw_pos_states_.resize(info_.joints.size(), 0.0);
    hw_pos_states_buffer_.resize(info_.joints.size(), 0.0); // 数据缓冲数组
    hw_vel_states_.resize(info_.joints.size(), 0.0);
    hw_pos_commands_.resize(info_.joints.size(), 0.0);
    last_sent_commands_.resize(info_.joints.size(), 0.0);

    // 索引映射：建立 URDF 关节名与数组下标的对应关系，方便后续快速查找
    // 以后想找 "Joint1_l" 的数据，直接查这个 map 就知道它在数组的第几个
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      joint_name_to_index_[info_.joints[i].name] = i;
    }

    // 启动后台线程：独立运行 ROS2 节点，不阻塞 ros2_control 的主实时循环
    spin_thread_ = std::thread([this]() {
      // 死循环，订阅 node_ ，包含所有订阅回调
      rclcpp::spin(node_);
    });
    // 线程独立
    spin_thread_.detach();

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // 消息格式处理 genie_msg/ArmState to 数组arr
  // 回调处理：将底层话题消息转换到内部状态数组
  // 传入两个参数，msg 为genie_msgs/msg/arm_state中的14个电机信息，suffix为字符串"_l"或者"_r"
  void process_arm_data(const genie_msgs::msg::ArmState::SharedPtr msg, const std::string & suffix)
  {
    // 加锁
    // lock_guard 这种锁会在析构时自动开锁
    std::lock_guard<std::mutex> lock(mutex_);
    // 遍历电机
    for (size_t i = 0; i < msg->motor_states.size(); ++i) {
      // 在本地拼出电机名称存在 expected_name 中
      std::string expected_name = "Joint" + std::to_string(i + 1) + suffix;
      if (joint_name_to_index_.count(expected_name)) {
        size_t idx = joint_name_to_index_[expected_name];
        double pos = msg->motor_states[i].position;
        if (std::isfinite(pos)) {
          // hw_pos_states_[idx] = pos;  // 写入状态数组，供 ros2_control 的 read() 函数读取
          hw_pos_states_buffer_[idx] = pos; // 写入缓冲区
          if (suffix == "_l") received_left_ = true;  // 自锁等待
          if (suffix == "_r") received_right_ = true;
        }
      }
    }
  }

  // 头部 HAL 反馈里的名字通常是 "head_yaw/head_pitch"，
  // 但 URDF/MoveIt 里的关节名是 "joint_head_yaw/joint_head_pitch"。
  // 如果这里不做映射，启动时头部反馈写不进 buffer，on_activate 会把 0 当成当前头部位置，
  // 接着 write() 就会把头部暴力发回 0。
  std::string normalize_head_joint_name(const std::string & raw_name) const
  {
    if (raw_name == "head_yaw" || raw_name == "yaw") {
      return "joint_head_yaw";
    }
    if (raw_name == "head_pitch" || raw_name == "pitch") {
      return "joint_head_pitch";
    }
    return raw_name;
  }

  // 必须在 mutex_ 已经上锁时调用
  void sync_head_command_to_feedback_locked()
  {
    const char * head_joints[] = {"joint_head_yaw", "joint_head_pitch"};
    for (const auto & joint_name : head_joints) {
      auto it = joint_name_to_index_.find(joint_name);
      if (it == joint_name_to_index_.end()) {
        continue;
      }
      const size_t idx = it->second;
      const double pos = hw_pos_states_buffer_[idx];
      if (std::isfinite(pos)) {
        hw_pos_states_[idx] = pos;
        hw_pos_commands_[idx] = pos;
        last_sent_commands_[idx] = pos;
      }
    }
  }

  // 仿 sync_head_command_to_feedback_locked：把腰部两个关节的"真实反馈"同步到命令，
  // 让启动/迟到时命令=当前实际位置，而不是默认 0。必须在 mutex_ 已上锁时调用。
  void sync_waist_command_to_feedback_locked()
  {
    const char * waist_joints[] = {"joint_body_pitch", "joint_lift_body"};
    for (const auto & joint_name : waist_joints) {
      auto it = joint_name_to_index_.find(joint_name);
      if (it == joint_name_to_index_.end()) {
        continue;
      }
      const size_t idx = it->second;
      const double pos = hw_pos_states_buffer_[idx];
      if (std::isfinite(pos)) {
        hw_pos_states_[idx] = pos;
        hw_pos_commands_[idx] = pos;
        last_sent_commands_[idx] = pos;
      }
    }
  }

  void process_head_data(const genie_msgs::msg::HeadState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    bool updated_any_head_joint = false;

    // 遍历头部消息中的电机名数组。兼容 HAL 名称和 URDF 名称。
    for (size_t i = 0; i < msg->name.size(); ++i) {
      std::string expected_urdf_name = normalize_head_joint_name(msg->name[i]);

      // 去字典里查这个关节在 hw_pos_states_buffer_ 中的下标
      if (joint_name_to_index_.count(expected_urdf_name)) {
        size_t idx = joint_name_to_index_[expected_urdf_name];

        // 防越界保护，提取位置信息
        if (i < msg->motor_states.size()) {
          double pos = msg->motor_states[i].position;
          if (std::isfinite(pos)) {
            hw_pos_states_buffer_[idx] = pos;
            updated_any_head_joint = true;

            if (expected_urdf_name == "joint_head_yaw") {
              received_head_yaw_ = true;
            } else if (expected_urdf_name == "joint_head_pitch") {
              received_head_pitch_ = true;
            }
          }
        }
      }
    }

    // 只有 yaw/pitch 都收到，才认为头部状态有效。
    // 这样可以避免只收到半包时启动控制，把另一个关节误认为 0。
    received_neck_ = (received_head_yaw_ && received_head_pitch_);

    // 如果硬件已经激活，但激活时没等到头部反馈，则第一次拿到完整头部反馈时再打开头部命令发布。
    if (is_initialized_ && received_neck_ && !head_command_enabled_) {
      sync_head_command_to_feedback_locked();
      head_command_enabled_ = true;
      RCLCPP_WARN(node_->get_logger(),
                  "Head feedback received after activation. Synced head command to current feedback and enabled head command publishing.");
    } else if (is_initialized_ && updated_any_head_joint && !received_neck_) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "Partial head feedback received, but yaw/pitch are not both ready. Head command publishing remains disabled.");
    }
  }





  // 处理腰部/躯干反馈
  void process_waist_data(const genie_msgs::msg::WaistState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // 腰反馈总带 name[]（已 echo /hal/waist_state 确认），直接按名字匹配（仿头部 process_head_data）。
    // 兼容 HAL 别名(waist_pitch/lift) 和 URDF 名(joint_body_pitch/joint_lift_body)。
    for (size_t i = 0; i < msg->name.size(); ++i) {
      std::string urdf_name;
      if (msg->name[i] == "waist_pitch" || msg->name[i] == "joint_body_pitch") {
        urdf_name = "joint_body_pitch";
      } else if (msg->name[i] == "lift" || msg->name[i] == "joint_lift_body") {
        urdf_name = "joint_lift_body";
      } else {
        urdf_name = msg->name[i];
      }

      if (joint_name_to_index_.count(urdf_name) && i < msg->motor_states.size()) {
        double pos = msg->motor_states[i].position;
        if (std::isfinite(pos)) {
          hw_pos_states_buffer_[joint_name_to_index_[urdf_name]] = pos;
          if (urdf_name == "joint_body_pitch") received_waist_pitch_ = true;
          else if (urdf_name == "joint_lift_body") received_waist_lift_ = true;
        }
      }
    }

    // 仿头部 received_neck_：pitch/lift 都收到才算腰 ready，避免半包数据骗过保护。
    received_waist_ = (received_waist_pitch_ && received_waist_lift_);

    // 迟到补救（仿头部 process_head_data）：激活时没等到腰反馈、之后才到的，
    // 第一次拿到腰反馈时把命令同步到真实位置并放开腰命令发布，避免一直被钉在 0。
    if (is_initialized_ && received_waist_ && !waist_command_enabled_) {
      sync_waist_command_to_feedback_locked();
      waist_command_enabled_ = true;
      RCLCPP_WARN(node_->get_logger(),
                  "Waist feedback received after activation. Synced waist command to feedback and enabled waist command publishing.");
    }
  }

  // ── 夹爪反馈换算 ──────────────────────────────────────────────
  // 实测(DEVLOG「话题测试」)：/wbc/*_ee_command 的 cmd=0.0 → /hal/*_ee_data 反馈 position=0.0°(全开)；
  //   cmd=1.0 → 反馈 position=120.0°(紧闭)。线性：raw_deg = frac * 120，frac∈[0,1](0=开,1=合)。
  static constexpr double kGripperRawOpenDeg   = 0.0;
  static constexpr double kGripperRawClosedDeg = 120.0;
  // 已用 rviz(robot_state_publisher/joint_state_publisher_gui) 三终端实测确认：
  //   left_narrow2_joint = 0.0 → 闭合；= -1.0 → 全开。跟最初假设正好相反，故为 false。
  static constexpr bool kNarrow2ZeroIsOpen = false;

  // 原始角度(0~120°) → narrow2_joint 弧度([-1,0])
  static double gripperRawDegToJointRad(double raw_deg)
  {
    const double frac = (raw_deg - kGripperRawOpenDeg) / (kGripperRawClosedDeg - kGripperRawOpenDeg);  // 0=开,1=合
    return kNarrow2ZeroIsOpen ? -frac : (frac - 1.0);
  }
  // narrow2_joint 弧度 → SDK cmd 用的 0~1 开合分数。给 write() 转发夹爪命令用
  // （只在命令值持续变化、即 JTC 真执行轨迹时才会被调用，见 write() 里的说明）。
  static double gripperJointRadToRawFrac(double rad)
  {
    return kNarrow2ZeroIsOpen ? -rad : (rad + 1.0);
  }

  // 必须在 mutex_ 已上锁时调用（仿 sync_waist_command_to_feedback_locked）
  void sync_gripper_command_to_feedback_locked()
  {
    const char* names[] = {"left_narrow2_joint", "right_narrow2_joint"};
    for (const auto& name : names) {
      auto it = joint_name_to_index_.find(name);
      if (it == joint_name_to_index_.end()) continue;
      const size_t idx = it->second;
      const double pos = hw_pos_states_buffer_[idx];
      if (std::isfinite(pos)) {
        hw_pos_states_[idx] = pos;
        hw_pos_commands_[idx] = pos;
        last_sent_commands_[idx] = pos;
      }
    }
  }

  // 夹爪反馈：更新 narrow2_joint 的状态供 MoveIt/碰撞检测读取。命令是否转发不在这里判断——
  //   由 write() 里"命令值有没有变"决定（只在 JTC 真执行轨迹时转发，见 DEVLOG「夹爪控制」）。
  void process_gripper_data(const genie_msgs::msg::EndState::SharedPtr msg, bool is_left)
  {
    if (msg->end_state.empty()) return;   // 夹爪单关节，取 end_state[0]
    std::lock_guard<std::mutex> lock(mutex_);
    const std::string joint_name = is_left ? "left_narrow2_joint" : "right_narrow2_joint";
    auto it = joint_name_to_index_.find(joint_name);
    if (it == joint_name_to_index_.end()) return;

    const double raw_deg = msg->end_state[0].position;
    if (!std::isfinite(raw_deg)) return;
    hw_pos_states_buffer_[it->second] = gripperRawDegToJointRad(raw_deg);
    (is_left ? received_left_gripper_ : received_right_gripper_) = true;

    const bool ready = received_left_gripper_ && received_right_gripper_;
    if (is_initialized_ && ready && !gripper_command_enabled_) {
      sync_gripper_command_to_feedback_locked();
      gripper_command_enabled_ = true;
      (is_left ? left_gripper_cmd_prev_ : right_gripper_cmd_prev_) = hw_pos_commands_[it->second];
      RCLCPP_WARN(node_->get_logger(),
                  "Gripper feedback received after activation. Synced gripper command to feedback.");
    }
  }


  // 接口暴露：将内部位置/速度数组挂载到 ros2_control 的资源池（只读状态）
  // 传出一个包含“状态接口”的列表
  // 此处并没有声明父类，但export_state_interfaces是ROS2官方父类，
  // 所有想当硬件驱动的程序，必须准备一个叫 export_state_interfaces 的函数，用来上交状态
  // 此处使用override来处理多态时函数名归属权，声明以下皆为重写，不会出现新函数
  // 强制约束数据格式<hardware_interface::StateInterface>
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
      // .emplace_back() 类似于 .append()
      // hardware_interface::StateInterface( 关节名, 所需数据eg.position , 物理地址 )
      // 当需要访问时，直接去访问物理地址，简化了信息交互流程
      state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_pos_states_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_vel_states_[i]));
    }
    return state_interfaces;
  }

  // 轨迹控制器会向这里写入目标值
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_pos_commands_[i]));
    }
    return command_interfaces;
  }

  // 激活函数：当控制器启动时执行，用于同步起始位置
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    auto req = std::make_shared<genie_msgs::srv::ArmControl::Request>();
    // 发送使能指令
    req->control = true;
    // 异步指令，电机通电
    if (left_client_->wait_for_service(std::chrono::seconds(1))) left_client_->async_send_request(req);
    if (right_client_->wait_for_service(std::chrono::seconds(1))) right_client_->async_send_request(req);
    
    // 自锁等待：确保在正式控制前收到了左右臂的反馈，防止机械臂猛闪（Snap）
    // 等待received_left_变为true才能控制
    int retry = 0;
    while (!(received_left_ && received_right_ && received_neck_ && received_waist_
             && received_left_gripper_ && received_right_gripper_) && retry++ < 50) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // 打印各部位反馈是否就绪 —— 休眠启动排查神器：一眼看出谁的 /hal 反馈没到。
    RCLCPP_WARN(node_->get_logger(),
                "Activation feedback ready? left=%d right=%d neck=%d waist=%d left_gripper=%d right_gripper=%d (retry=%d)",
                received_left_, received_right_, received_neck_, received_waist_,
                received_left_gripper_, received_right_gripper_, retry);


    // 小范围上锁,将后台线程写入 buffer 的“真实硬件位置”安全地同步过来
    {
      std::lock_guard<std::mutex> lock(mutex_);
      hw_pos_states_ = hw_pos_states_buffer_;

      // 同步指令：将“上次发出的指令”设为“当前真实位置”，实现无缝平滑启动
      // 防止启动时瞬间复位
      for (size_t i = 0; i < hw_pos_states_.size(); i++) {
        hw_pos_commands_[i] = hw_pos_states_[i];
        last_sent_commands_[i] = hw_pos_states_[i];
      }

      // 头部单独加保险：只有完整收到 yaw/pitch 反馈后，才允许 write() 发布头部命令。
      // 否则启动时如果头部实际在 27deg，而 buffer 还是默认 0，会被强行拉回 0。
      if (received_neck_) {
        sync_head_command_to_feedback_locked();
        head_command_enabled_ = true;
      } else {
        head_command_enabled_ = false;
        RCLCPP_ERROR(node_->get_logger(),
                     "Head feedback was not ready during activation. Head command publishing is disabled until valid yaw+pitch feedback arrives.");
      }

      // 腰部同样加保险（仿头部）：只有收到腰反馈、命令已同步成真实位置，才允许 write() 发腰命令。
      // 否则休眠启动时 buffer 还是默认 0，会把腰暴力打回 0。
      if (received_waist_) {
        sync_waist_command_to_feedback_locked();
        waist_command_enabled_ = true;
      } else {
        waist_command_enabled_ = false;
        RCLCPP_ERROR(node_->get_logger(),
                     "Waist feedback was not ready during activation. Waist command publishing is disabled until valid feedback arrives.");
      }

      // 夹爪同样加保险：命令值同步成真实位置，且把"上一次命令值"也钉在这（NaN→真实值），
      // 这样第一次 write() 比较时 diff=0，不会把"启动同步"误判成"正在执行轨迹"而误发一次。
      if (received_left_gripper_ && received_right_gripper_) {
        sync_gripper_command_to_feedback_locked();
        gripper_command_enabled_ = true;
        if (joint_name_to_index_.count("left_narrow2_joint"))
          left_gripper_cmd_prev_ = hw_pos_commands_[joint_name_to_index_["left_narrow2_joint"]];
        if (joint_name_to_index_.count("right_narrow2_joint"))
          right_gripper_cmd_prev_ = hw_pos_commands_[joint_name_to_index_["right_narrow2_joint"]];
      } else {
        gripper_command_enabled_ = false;
        RCLCPP_ERROR(node_->get_logger(),
                     "Gripper feedback was not ready during activation.");
      }
    }

    is_initialized_ = true;
    return hardware_interface::CallbackReturn::SUCCESS;
  }





  // 读取
  // 在每个控制周期开始时调用
  hardware_interface::return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    // 先加锁
    std::lock_guard<std::mutex> lock(mutex_);
    // 
    hw_pos_states_ = hw_pos_states_buffer_;
    return hardware_interface::return_type::OK;
  }



  // 写入：每个控制周期结束时调用，执行平滑逻辑并下发
  hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    if (!is_initialized_) return hardware_interface::return_type::OK;

    // 准备要发的消息，JointState 格式
    auto msg_arm = std::make_shared<sensor_msgs::msg::JointState>();
    auto msg_head = std::make_shared<sensor_msgs::msg::JointState>();
    auto msg_waist = std::make_shared<sensor_msgs::msg::JointState>();
    auto msg_left_ee = std::make_shared<sensor_msgs::msg::JointState>();
    auto msg_right_ee = std::make_shared<sensor_msgs::msg::JointState>();

    msg_arm->header.stamp = node_->now();
    msg_head->header.stamp = node_->now();
    msg_waist->header.stamp = node_->now();
    msg_left_ee->header.stamp = node_->now();
    msg_right_ee->header.stamp = node_->now();

    // 第一步：全局平滑滤波更新（保证所有关节都得到合理的步进限制）
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      double diff = hw_pos_commands_[i] - last_sent_commands_[i];
      if (std::abs(diff) > 1e-4) {
        last_sent_commands_[i] += std::clamp(diff, -STEP_SIZE, STEP_SIZE);
      } else {
        last_sent_commands_[i] = hw_pos_commands_[i];
      }

      std::string urdf_name = info_.joints[i].name;

      // 第二步：只在此处打包手臂和头部（它们通常走标准的名称匹配或者独立的底层解析）
      if (urdf_name.length() >= 7 && (urdf_name.back() == 'l' || urdf_name.back() == 'r')) {
        char side = urdf_name.back();
        std::string num = urdf_name.substr(5, urdf_name.length() - 7);
        std::string wbc_name = (side == 'l') ? "left_arm_joint_" + num : "right_arm_joint_" + num;
        msg_arm->name.push_back(wbc_name);
        msg_arm->position.push_back(last_sent_commands_[i]);
      }
      else if (urdf_name == "joint_head_yaw" || urdf_name == "joint_head_pitch") {
        // 启动阶段必须等头部真实反馈同步完成后才允许发布头部命令，
        // 防止把默认 0 当作目标值导致头部突然回零。
        // control_head_=false 时整条头命令都不发(交给外部 PID)，但上面 read/回调仍照常更新头部状态。
        if (control_head_ && head_command_enabled_) {
          std::string wbc_name = urdf_name.substr(6);
          msg_head->name.push_back(wbc_name);
          msg_head->position.push_back(last_sent_commands_[i]);
        }
      }
      else if (urdf_name == "left_narrow2_joint" || urdf_name == "right_narrow2_joint") {
        // 夹爪：只在 JTC 真正插值执行(命令值持续变化)时才转发，空闲悬停(不变)就不发——
        //   这样不会跟 setGripper()/GripperController 的 50Hz 持续刷流抢话题。
        //   （判据只看"变没变"，不区分谁在变；真出现"MoveIt 执行的同时又手动 setGripper()"
        //     这种同时抢的情况，跟两个人同时按同一个开关一样，本就无法两全，不特殊处理。）
        const bool is_left = (urdf_name == "left_narrow2_joint");
        double& prev = is_left ? left_gripper_cmd_prev_ : right_gripper_cmd_prev_;
        const double cur = last_sent_commands_[i];
        const bool changing = std::isfinite(prev) && std::abs(cur - prev) > 1e-4;
        if (gripper_command_enabled_ && changing) {
          const double raw_frac = std::clamp(gripperJointRadToRawFrac(cur), 0.0, 1.0);
          auto& m = is_left ? msg_left_ee : msg_right_ee;
          m->name = { is_left ? "left_gripper" : "right_gripper" };
          m->position = { raw_frac };
        }
        prev = cur;
      }
    }
    
    // 第三步：打包腰部/躯干关节。★加 waist_command_enabled_ 守卫★：腰反馈没同步好就不发，
    //   防止休眠启动时用默认 0 把腰打回零（仿头部 head_command_enabled_）。
    if (waist_command_enabled_ &&
        joint_name_to_index_.count("joint_lift_body") && joint_name_to_index_.count("joint_body_pitch")) {
      size_t lift_idx = joint_name_to_index_["joint_lift_body"];
      size_t pitch_idx = joint_name_to_index_["joint_body_pitch"];

      // 【核心修复点】
      // 既然目前“调 pitch 改变了高度”，意味着底层 WBC 盲认 msg_waist->position[0] 是高度。
      // 那我们就顺从它的硬编码逻辑，把 MoveIt 的 lift 关节死死钉在第 0 位，pitch 钉在第 1 位：
      msg_waist->name.push_back("joint_body_pitch");
      msg_waist->position.push_back(last_sent_commands_[pitch_idx]);

      msg_waist->name.push_back("joint_lift_body");
      msg_waist->position.push_back(last_sent_commands_[lift_idx]);
    }
    
    // 4. 硬件级高频心跳发布
    if (!msg_arm->name.empty())  pub_arm_command_->publish(*msg_arm);
    if (!msg_head->name.empty()) pub_head_command_->publish(*msg_head);
    if (!msg_waist->name.empty()) pub_waist_command_->publish(*msg_waist);
    // 夹爪：name 非空 = 上面判定"正在变化"才会填，空闲时天然不发，见上方注释。
    if (!msg_left_ee->name.empty())  pub_left_ee_command_->publish(*msg_left_ee);
    if (!msg_right_ee->name.empty()) pub_right_ee_command_->publish(*msg_right_ee);

    return hardware_interface::return_type::OK;
  }





private:
  std::shared_ptr<rclcpp::Node> node_;
  std::thread spin_thread_;
  bool is_initialized_ = false;

  bool received_left_ = false;
  bool received_right_ = false;
  bool received_neck_ = false;
  bool received_waist_ = false;

  bool received_head_yaw_ = false;
  bool received_head_pitch_ = false;
  bool head_command_enabled_ = false;
  bool control_head_ = true;             // 头部开关(URDF param control_head)：false=硬件层不发头命令

  bool received_waist_pitch_ = false;    // 仿头部逐关节确认：pitch 反馈真到了吗
  bool received_waist_lift_ = false;     //                    lift  反馈真到了吗
  bool waist_command_enabled_ = false;   // 腰反馈没同步前，不发腰命令，防启动猛回 0

  bool received_left_gripper_ = false;   // 夹爪反馈：一直接着，供 MoveIt 碰撞检测用
  bool received_right_gripper_ = false;
  bool gripper_command_enabled_ = false;

  // "只在真正执行轨迹时转发夹爪命令"的判据：跟上一周期的命令值比，变了才发。
  //   NaN=还没同步过初值（on_activate 里会赋成真实值，第一次 write() 就不会误判成"在变"）。
  double left_gripper_cmd_prev_  = std::numeric_limits<double>::quiet_NaN();
  double right_gripper_cmd_prev_ = std::numeric_limits<double>::quiet_NaN();

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_arm_command_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_head_command_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_waist_command_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_left_ee_command_, pub_right_ee_command_;

  rclcpp::Subscription<genie_msgs::msg::ArmState>::SharedPtr sub_left_, sub_right_;
  rclcpp::Subscription<genie_msgs::msg::HeadState>::SharedPtr sub_neck_;
  rclcpp::Subscription<genie_msgs::msg::WaistState>::SharedPtr sub_waist_;
  rclcpp::Subscription<genie_msgs::msg::EndState>::SharedPtr sub_left_ee_, sub_right_ee_;


  rclcpp::Client<genie_msgs::srv::ArmControl>::SharedPtr left_client_, right_client_;

  std::map<std::string, size_t> joint_name_to_index_;
  std::vector<double> hw_pos_states_;
  std::vector<double> hw_pos_states_buffer_;  // 子线程写入缓冲区
  
  std::vector<double> hw_vel_states_;
  std::vector<double> hw_pos_commands_;
  std::vector<double> last_sent_commands_; 

  std::mutex mutex_;  // 全局互斥锁 
};

} // namespace genie_hardware

// 插件宏：允许 ros2_control 动态发现并加载这个类
PLUGINLIB_EXPORT_CLASS(genie_hardware::GenieArmHardware, hardware_interface::SystemInterface)