#include "exoskeleton_controllers/exo_impedance_controller.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace exoskeleton_controllers
{

// ===================== on_init =====================

controller_interface::CallbackReturn ExoImpedanceController::on_init()
{
  try
  {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::vector<double>>("default_kp", {});
    auto_declare<std::vector<double>>("default_kd", {});
    auto_declare<std::vector<double>>("default_tube_half_width", {});
    auto_declare<std::vector<double>>("default_tube_transition_width", {});
    auto_declare<std::vector<double>>("torque_limit", {});
    auto_declare<std::vector<double>>("torque_rate_limit", {});
    auto_declare<double>("velocity_filter_alpha", 0.2);
    auto_declare<double>("data_timeout", 0.01);
    auto_declare<double>("ref_scale", 1.0);
    auto_declare<double>("tuning_command_timeout", 0.25);
    auto_declare<double>("auxiliary_command_timeout", 0.1);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "参数声明失败: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

// ===================== on_configure =====================

controller_interface::CallbackReturn ExoImpedanceController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  default_kp_ = get_node()->get_parameter("default_kp").as_double_array();
  default_kd_ = get_node()->get_parameter("default_kd").as_double_array();
  default_tube_half_width_ =
    get_node()->get_parameter("default_tube_half_width").as_double_array();
  default_tube_transition_width_ =
    get_node()->get_parameter("default_tube_transition_width").as_double_array();
  torque_limit_ = get_node()->get_parameter("torque_limit").as_double_array();
  torque_rate_limit_ = get_node()->get_parameter("torque_rate_limit").as_double_array();
  velocity_filter_alpha_ = get_node()->get_parameter("velocity_filter_alpha").as_double();
  data_timeout_ = get_node()->get_parameter("data_timeout").as_double();
  ref_scale_ = get_node()->get_parameter("ref_scale").as_double();
  tuning_command_timeout_ = get_node()->get_parameter("tuning_command_timeout").as_double();
  auxiliary_command_timeout_ =
    get_node()->get_parameter("auxiliary_command_timeout").as_double();

  num_joints_ = joint_names_.size();

  if (num_joints_ == 0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "未指定关节");
    return controller_interface::CallbackReturn::ERROR;
  }

  // 参考接口存储: 每关节 5 个 (target_position, target_velocity, feedforward_torque, kp, kd)
  // 必须与 on_export_reference_interfaces() 导出的数量一致, 否则链式控制器配置失败
  reference_interfaces_.resize(num_joints_ * 5);

  // 参数维度校验
  if (default_kp_.size() != num_joints_ || default_kd_.size() != num_joints_ ||
      default_tube_half_width_.size() != num_joints_ ||
      default_tube_transition_width_.size() != num_joints_ ||
      torque_limit_.size() != num_joints_ || torque_rate_limit_.size() != num_joints_)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "参数维度不匹配: joints=%zu, kp=%zu, kd=%zu, tube=%zu, transition=%zu, limit=%zu, rate=%zu",
      num_joints_, default_kp_.size(), default_kd_.size(),
      default_tube_half_width_.size(), default_tube_transition_width_.size(),
      torque_limit_.size(), torque_rate_limit_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (std::any_of(
      default_tube_half_width_.begin(), default_tube_half_width_.end(),
      [](double value) {return !std::isfinite(value) || value < 0.0;}) ||
    std::any_of(
      default_tube_transition_width_.begin(), default_tube_transition_width_.end(),
      [](double value) {return !std::isfinite(value) || value < 0.0;}))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "虚拟管道半宽和平滑过渡宽度必须为有限非负数");
    return controller_interface::CallbackReturn::ERROR;
  }

  // 初始化内部状态
  qdot_filt_.assign(num_joints_, 0.0);
  tau_prev_.assign(num_joints_, 0.0);

  // 计算参考接口偏移量（每关节 5 个接口）
  ref_offsets_.resize(num_joints_);
  for (size_t i = 0; i < num_joints_; ++i)
  {
    ref_offsets_[i].target_position = i * 5 + 0;
    ref_offsets_[i].target_velocity = i * 5 + 1;
    ref_offsets_[i].feedforward_torque = i * 5 + 2;
    ref_offsets_[i].kp = i * 5 + 3;
    ref_offsets_[i].kd = i * 5 + 4;
  }

  // 创建订阅者（非链式模式下使用）
  ref_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
    "/cpg/joint_ref", rclcpp::SystemDefaultsQoS(),
    [this](const sensor_msgs::msg::JointState::SharedPtr msg)
    {
      // 将 CPG 参考写入 reference_interfaces_
      for (size_t i = 0; i < num_joints_; ++i)
      {
        for (size_t j = 0; j < msg->name.size(); ++j)
        {
          if (msg->name[j] == joint_names_[i])
          {
            reference_interfaces_[ref_offsets_[i].target_position] = msg->position[j] * ref_scale_;
            if (j < msg->velocity.size())
            {
              reference_interfaces_[ref_offsets_[i].target_velocity] = msg->velocity[j] * ref_scale_;
            }
            break;
          }
        }
      }
      last_ref_time_ = get_node()->get_clock()->now();
      ref_data_valid_ = true;
    });

  // 创建调试发布器 (200Hz): 控制误差/力矩/时序可视化
  debug_pub_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
    "/impedance/debug", rclcpp::SensorDataQoS());
  timing_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/impedance/timing", rclcpp::SensorDataQoS());
  component_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/impedance/components", rclcpp::SensorDataQoS());

  tuning_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/impedance_tuning/command", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
      if (msg->data.empty())
      {
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "忽略空调参命令");
        return;
      }
      TuningCommand command;
      command.session_active = true;
      command.kp = default_kp_;
      command.kd = default_kd_;
      command.tube_half_width = default_tube_half_width_;
      command.tube_transition_width = default_tube_transition_width_;
      if (msg->data[0] == 3.0)
      {
        const size_t expected_size = 3 + 4 * num_joints_;
        if (msg->data.size() != expected_size || !std::isfinite(msg->data[1]) ||
          msg->data[1] < 0.0 || !std::isfinite(msg->data[2]) || msg->data[2] < 0.0)
        {
          RCLCPP_WARN_THROTTLE(
            get_node()->get_logger(), *get_node()->get_clock(), 1000,
            "忽略无效V3调参命令: 收到%zu项, 期望%zu项", msg->data.size(), expected_size);
          return;
        }
        command.enabled_mask = static_cast<uint64_t>(msg->data[1]);
        command.zero_sequence = static_cast<uint64_t>(msg->data[2]);
        for (size_t i = 0; i < num_joints_; ++i)
        {
          const double kp = msg->data[3 + 4 * i];
          const double kd = msg->data[4 + 4 * i];
          const double tube = msg->data[5 + 4 * i];
          const double transition = msg->data[6 + 4 * i];
          if (!std::isfinite(kp) || !std::isfinite(kd) || !std::isfinite(tube) ||
            !std::isfinite(transition) || kp < 0.0 || kd < 0.0 || tube < 0.0 ||
            transition < 0.0)
          {
            RCLCPP_WARN_THROTTLE(
              get_node()->get_logger(), *get_node()->get_clock(), 1000,
              "忽略无效V3管道参数: joint=%zu", i);
            return;
          }
          command.kp[i] = kp;
          command.kd[i] = kd;
          command.tube_half_width[i] = tube;
          command.tube_transition_width[i] = transition;
        }
      }
      else if (msg->data[0] == 2.0)
      {
        const size_t expected_size = 3 + 2 * num_joints_;
        if (msg->data.size() != expected_size || !std::isfinite(msg->data[1]) ||
          msg->data[1] < 0.0 || !std::isfinite(msg->data[2]) || msg->data[2] < 0.0)
        {
          RCLCPP_WARN_THROTTLE(
            get_node()->get_logger(), *get_node()->get_clock(), 1000,
            "忽略无效V2调参命令: 收到%zu项, 期望%zu项", msg->data.size(), expected_size);
          return;
        }
        command.enabled_mask = static_cast<uint64_t>(msg->data[1]);
        command.zero_sequence = static_cast<uint64_t>(msg->data[2]);
        for (size_t i = 0; i < num_joints_; ++i)
        {
          const double kp = msg->data[3 + 2 * i];
          const double kd = msg->data[4 + 2 * i];
          if (!std::isfinite(kp) || !std::isfinite(kd) || kp < 0.0 || kd < 0.0)
          {
            RCLCPP_WARN_THROTTLE(
              get_node()->get_logger(), *get_node()->get_clock(), 1000,
              "忽略无效V2增益: joint=%zu kp=%.3f kd=%.3f", i, kp, kd);
            return;
          }
          command.kp[i] = kp;
          command.kd[i] = kd;
        }
      }
      else
      {
        // 兼容旧版单关节GUI: [output_enabled, joint_index, kp, kd, zero_sequence]。
        if (msg->data.size() < 4 || !std::isfinite(msg->data[1]) ||
          msg->data[1] < 0.0 || static_cast<size_t>(msg->data[1]) >= num_joints_ ||
          !std::isfinite(msg->data[2]) || !std::isfinite(msg->data[3]) ||
          msg->data[2] < 0.0 || msg->data[3] < 0.0)
        {
          RCLCPP_WARN_THROTTLE(
            get_node()->get_logger(), *get_node()->get_clock(), 1000,
            "忽略无效旧版调参命令");
          return;
        }
        const size_t joint_index = static_cast<size_t>(msg->data[1]);
        command.enabled_mask = msg->data[0] > 0.5 ? (uint64_t{1} << joint_index) : 0U;
        command.kp[joint_index] = msg->data[2];
        command.kd[joint_index] = msg->data[3];
        command.zero_sequence = msg->data.size() >= 5 && msg->data[4] > 0.0 ?
          static_cast<uint64_t>(msg->data[4]) : 0U;
      }
      command.stamp_ns = get_node()->get_clock()->now().nanoseconds();
      tuning_command_buffer_.writeFromNonRT(command);
    });

  auxiliary_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/impedance/auxiliary_command", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
      const size_t expected_size = 3 + 2 * num_joints_;
      if (msg->data.size() != expected_size || msg->data[0] != 1.0)
      {
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "忽略无效补偿命令: 收到%zu项, 期望V1的%zu项", msg->data.size(), expected_size);
        return;
      }
      AuxiliaryCommand command;
      command.received = true;
      command.transparency_enabled = msg->data[1] > 0.5;
      command.assist_enabled = msg->data[2] > 0.5;
      command.transparency_iq.assign(num_joints_, 0.0);
      command.assist_iq.assign(num_joints_, 0.0);
      for (size_t i = 0; i < num_joints_; ++i)
      {
        const double transparency = msg->data[3 + i];
        const double assist = msg->data[3 + num_joints_ + i];
        if (!std::isfinite(transparency) || !std::isfinite(assist))
        {
          RCLCPP_WARN_THROTTLE(
            get_node()->get_logger(), *get_node()->get_clock(), 1000,
            "忽略含非有限值的补偿命令: joint=%zu", i);
          return;
        }
        command.transparency_iq[i] = transparency;
        command.assist_iq[i] = assist;
      }
      command.stamp_ns = get_node()->get_clock()->now().nanoseconds();
      auxiliary_command_buffer_.writeFromNonRT(command);
    });

  last_ref_time_ = get_node()->get_clock()->now();

  RCLCPP_INFO(
    get_node()->get_logger(), "阻抗控制器已配置: %zu 关节, 默认 Kp=[%s], Kd=[%s]",
    num_joints_,
    [this]() {
      std::string s;
      for (size_t i = 0; i < num_joints_; ++i)
        s += (i ? "," : "") + std::to_string(static_cast<int>(default_kp_[i]));
      return s;
    }().c_str(),
    [this]() {
      std::string s;
      for (size_t i = 0; i < num_joints_; ++i)
        s += (i ? "," : "") + std::to_string(static_cast<int>(default_kd_[i]));
      return s;
    }().c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

// ===================== on_activate =====================

controller_interface::CallbackReturn ExoImpedanceController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 初始化参考接口为默认值
  for (size_t i = 0; i < num_joints_; ++i)
  {
    reference_interfaces_[ref_offsets_[i].target_position] = 0.0;
    reference_interfaces_[ref_offsets_[i].target_velocity] = 0.0;
    reference_interfaces_[ref_offsets_[i].feedforward_torque] = 0.0;
    reference_interfaces_[ref_offsets_[i].kp] = default_kp_[i];
    reference_interfaces_[ref_offsets_[i].kd] = default_kd_[i];
  }

  qdot_filt_.assign(num_joints_, 0.0);
  tau_prev_.assign(num_joints_, 0.0);
  ref_data_valid_ = false;
  zero_torque_ = true;  // 激活后先输出零力矩, 收到有效参考后自动恢复
  tuning_command_buffer_.writeFromNonRT(TuningCommand{});
  auxiliary_command_buffer_.writeFromNonRT(AuxiliaryCommand{});
  applied_zero_sequence_ = 0;

  RCLCPP_INFO(get_node()->get_logger(), "阻抗控制器已激活");
  return controller_interface::CallbackReturn::SUCCESS;
}

// ===================== on_deactivate =====================

controller_interface::CallbackReturn ExoImpedanceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 归零力矩
  for (auto & iface : command_interfaces_)
  {
    iface.set_value(0.0);
  }
  tau_prev_.assign(num_joints_, 0.0);

  RCLCPP_INFO(get_node()->get_logger(), "阻抗控制器已去激活，力矩归零");
  return controller_interface::CallbackReturn::SUCCESS;
}

// ===================== 接口配置 =====================

controller_interface::InterfaceConfiguration
ExoImpedanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // 每关节一个 effort 命令接口，随后是手动软件清零脉冲接口。
  for (const auto & joint : joint_names_)
  {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  for (const auto & joint : joint_names_)
  {
    config.names.push_back(joint + "/zero_position");
  }

  return config;
}

controller_interface::InterfaceConfiguration
ExoImpedanceController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // 每关节 position + velocity 状态接口
  for (const auto & joint : joint_names_)
  {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }

  return config;
}

// ===================== 导出参考接口 =====================

std::vector<hardware_interface::CommandInterface>
ExoImpedanceController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(num_joints_ * 5);

  // 参考接口名必须以控制器节点名开头 (ros2_control 强制校验 prefix_name),
  // 否则链式控制器无法导出参考接口 → 配置失败
  const std::string controller_prefix = get_node()->get_name();

  for (size_t i = 0; i < num_joints_; ++i)
  {
    const auto & joint = joint_names_[i];
    interfaces.emplace_back(
      controller_prefix, joint + "/target_position",
      &reference_interfaces_[ref_offsets_[i].target_position]);
    interfaces.emplace_back(
      controller_prefix, joint + "/target_velocity",
      &reference_interfaces_[ref_offsets_[i].target_velocity]);
    interfaces.emplace_back(
      controller_prefix, joint + "/feedforward_torque",
      &reference_interfaces_[ref_offsets_[i].feedforward_torque]);
    interfaces.emplace_back(
      controller_prefix, joint + "/kp",
      &reference_interfaces_[ref_offsets_[i].kp]);
    interfaces.emplace_back(
      controller_prefix, joint + "/kd",
      &reference_interfaces_[ref_offsets_[i].kd]);
  }

  return interfaces;
}

// ===================== 链式模式切换 =====================

bool ExoImpedanceController::on_set_chained_mode(bool chained_mode)
{
  // 链式模式下禁用订阅者，避免并发写入
  if (chained_mode)
  {
    ref_sub_.reset();
    RCLCPP_INFO(get_node()->get_logger(), "切换到链式模式，订阅者已禁用");
  }
  else
  {
    // 重新创建订阅者
    ref_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      "/cpg/joint_ref", rclcpp::SystemDefaultsQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg)
      {
        for (size_t i = 0; i < num_joints_; ++i)
        {
          for (size_t j = 0; j < msg->name.size(); ++j)
          {
            if (msg->name[j] == joint_names_[i])
            {
              reference_interfaces_[ref_offsets_[i].target_position] = msg->position[j];
              if (j < msg->velocity.size())
              {
                reference_interfaces_[ref_offsets_[i].target_velocity] = msg->velocity[j];
              }
              break;
            }
          }
        }
        last_ref_time_ = get_node()->get_clock()->now();
        ref_data_valid_ = true;
      });
    RCLCPP_INFO(get_node()->get_logger(), "切换到独立模式，订阅者已启用");
  }

  return true;
}

// ===================== 从订阅者更新参考 =====================

controller_interface::return_type ExoImpedanceController::update_reference_from_subscribers()
{
  // 检查数据新鲜度 (必须收到过有效参考且在超时窗口内)
  auto now = get_node()->get_clock()->now();
  const bool fresh = ref_data_valid_ &&
    (now - last_ref_time_).seconds() <= data_timeout_;

  if (!fresh)
  {
    // 超时或从未收到数据: 输出零力矩 (安全默认), 保留参考接口原值, 数据恢复后自动继续
    if (ref_data_valid_)
    {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "参考数据超时 (>%.0fms)，输出零力矩", data_timeout_ * 1000);
      ref_data_valid_ = false;
    }
    zero_torque_ = true;
  }
  else
  {
    zero_torque_ = false;
    ref_data_valid_ = true;
  }

  return controller_interface::return_type::OK;
}

// ===================== 核心控制计算 =====================

controller_interface::return_type ExoImpedanceController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const double dt = period.seconds();
  const TuningCommand & tuning_command = *tuning_command_buffer_.readFromRT();
  const double tuning_age = 1e-9 * static_cast<double>(
    time.nanoseconds() - tuning_command.stamp_ns);
  const bool tuning_fresh = tuning_command.session_active && tuning_age >= 0.0 &&
    tuning_age <= tuning_command_timeout_;
  const AuxiliaryCommand & auxiliary_command = *auxiliary_command_buffer_.readFromRT();
  const double auxiliary_age = 1e-9 * static_cast<double>(
    time.nanoseconds() - auxiliary_command.stamp_ns);
  const bool auxiliary_fresh = auxiliary_command.received && auxiliary_age >= 0.0 &&
    auxiliary_age <= auxiliary_command_timeout_;
  for (size_t i = 0; i < num_joints_; ++i)
  {
    command_interfaces_[num_joints_ + i].set_value(0.0);
  }
  bool all_outputs_zero = true;
  std::vector<double> tube_components(num_joints_, 0.0);
  std::vector<double> transparency_components(num_joints_, 0.0);
  std::vector<double> assist_components(num_joints_, 0.0);
  std::vector<double> total_components(num_joints_, 0.0);

  for (size_t i = 0; i < num_joints_; ++i)
  {
    // Step 1: 从状态接口读取当前关节位置和速度
    const double q_current = state_interfaces_[2 * i].get_value();      // position
    const double qdot_raw = state_interfaces_[2 * i + 1].get_value();   // velocity

    // 速度低通滤波
    qdot_filt_[i] = velocity_filter_alpha_ * qdot_raw +
                    (1.0 - velocity_filter_alpha_) * qdot_filt_[i];

    // Step 2: 从参考接口读取目标值（由上游控制器写入或订阅者更新）
    const double q_ref = reference_interfaces_[ref_offsets_[i].target_position];
    const double qdot_ref = reference_interfaces_[ref_offsets_[i].target_velocity];
    const double tau_ff = reference_interfaces_[ref_offsets_[i].feedforward_torque];
    double kp = reference_interfaces_[ref_offsets_[i].kp];
    double kd = reference_interfaces_[ref_offsets_[i].kd];
    double tube_half_width = default_tube_half_width_[i];
    double tube_transition_width = default_tube_transition_width_[i];

    // Step 3: 虚拟管道壁阻抗 + 预留透明度/助力叠加量 + 上游前馈。
    double tau_target = 0.0;
    bool joint_enabled = !zero_torque_;
    if (tuning_command.session_active)
    {
      joint_enabled = tuning_fresh &&
        (tuning_command.enabled_mask & (uint64_t{1} << i)) != 0U;
      if (i < tuning_command.kp.size() && i < tuning_command.kd.size())
      {
        kp = tuning_command.kp[i];
        kd = tuning_command.kd[i];
      }
      if (i < tuning_command.tube_half_width.size() &&
        i < tuning_command.tube_transition_width.size())
      {
        tube_half_width = tuning_command.tube_half_width[i];
        tube_transition_width = tuning_command.tube_transition_width[i];
      }
    }
    if (joint_enabled)
    {
      const double pos_error = q_ref - q_current;
      const double vel_error = qdot_ref - qdot_filt_[i];
      const double outside_distance = std::abs(pos_error) - tube_half_width;
      double wall_activation = tube_half_width <= 0.0 ? 1.0 : 0.0;
      double wall_error = tube_half_width <= 0.0 ? pos_error : 0.0;
      if (tube_half_width > 0.0 && outside_distance > 0.0)
      {
        wall_error = std::copysign(outside_distance, pos_error);
        if (tube_transition_width <= 0.0)
        {
          wall_activation = 1.0;
        }
        else
        {
          const double normalized = std::clamp(
            outside_distance / tube_transition_width, 0.0, 1.0);
          wall_activation = normalized * normalized * (3.0 - 2.0 * normalized);
        }
      }
      const double tube_iq = wall_activation * (kp * wall_error + kd * vel_error);
      const double transparency_iq =
        auxiliary_fresh && auxiliary_command.transparency_enabled &&
        i < auxiliary_command.transparency_iq.size() ?
        auxiliary_command.transparency_iq[i] : 0.0;
      const double assist_iq =
        auxiliary_fresh && auxiliary_command.assist_enabled &&
        i < auxiliary_command.assist_iq.size() ? auxiliary_command.assist_iq[i] : 0.0;
      tube_components[i] = tube_iq;
      transparency_components[i] = transparency_iq;
      assist_components[i] = assist_iq;
      tau_target = tube_iq + transparency_iq + assist_iq + tau_ff;
    }

    // 变化率限制 (含超时归零: 平滑下降到 0)
    const double max_delta = torque_rate_limit_[i] * dt;
    double delta = tau_target - tau_prev_[i];
    delta = std::clamp(delta, -max_delta, max_delta);
    double tau = tau_prev_[i] + delta;

    // 力矩限幅
    tau = std::clamp(tau, -torque_limit_[i], torque_limit_[i]);

    tau_prev_[i] = tau;
    total_components[i] = tau;
    all_outputs_zero = all_outputs_zero && std::abs(tau) < 1e-9;

    // 写入命令接口
    command_interfaces_[i].set_value(tau);
  }

  // 只有调参输出已关闭且四关节命令均归零后，才向硬件层发送一次清零脉冲。
  if (tuning_fresh && tuning_command.enabled_mask == 0U && all_outputs_zero &&
    tuning_command.zero_sequence > applied_zero_sequence_)
  {
    for (size_t i = 0; i < num_joints_; ++i)
    {
      command_interfaces_[num_joints_ + i].set_value(1.0);
    }
    applied_zero_sequence_ = tuning_command.zero_sequence;
    RCLCPP_INFO(get_node()->get_logger(), "已请求硬件将四关节当前位置设为零位");
  }

  // 发布调试信息 (200Hz): JointState 编码 position=q_ref, velocity=q_measured, effort=tau
  auto dbg = sensor_msgs::msg::JointState();
  dbg.header.stamp = time;
  for (size_t i = 0; i < num_joints_; ++i)
  {
    dbg.name.push_back(joint_names_[i]);
    dbg.position.push_back(reference_interfaces_[ref_offsets_[i].target_position]);  // q_ref
    dbg.velocity.push_back(state_interfaces_[2 * i].get_value());                     // q_measured
    dbg.effort.push_back(tau_prev_[i]);                                               // 命令力矩
  }
  debug_pub_->publish(dbg);

  // 时序: [控制周期 s, 参考陈旧度 s] (延迟指标)
  auto tmg = std_msgs::msg::Float64MultiArray();
  tmg.data = {period.seconds(), (time - last_ref_time_).seconds()};
  timing_pub_->publish(tmg);

  auto components = std_msgs::msg::Float64MultiArray();
  components.data.reserve(4 * num_joints_);
  components.data.insert(
    components.data.end(), tube_components.begin(), tube_components.end());
  components.data.insert(
    components.data.end(), transparency_components.begin(), transparency_components.end());
  components.data.insert(
    components.data.end(), assist_components.begin(), assist_components.end());
  components.data.insert(
    components.data.end(), total_components.begin(), total_components.end());
  component_pub_->publish(components);

  return controller_interface::return_type::OK;
}

}  // namespace exoskeleton_controllers

// ===================== Pluginlib 注册 =====================

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  exoskeleton_controllers::ExoImpedanceController,
  controller_interface::ChainableControllerInterface)
