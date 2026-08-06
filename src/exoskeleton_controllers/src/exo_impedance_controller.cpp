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
    auto_declare<std::vector<double>>("torque_limit", {});
    auto_declare<std::vector<double>>("torque_rate_limit", {});
    auto_declare<double>("velocity_filter_alpha", 0.2);
    auto_declare<double>("data_timeout", 0.01);
    auto_declare<double>("ref_scale", 1.0);
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
  torque_limit_ = get_node()->get_parameter("torque_limit").as_double_array();
  torque_rate_limit_ = get_node()->get_parameter("torque_rate_limit").as_double_array();
  velocity_filter_alpha_ = get_node()->get_parameter("velocity_filter_alpha").as_double();
  data_timeout_ = get_node()->get_parameter("data_timeout").as_double();
  ref_scale_ = get_node()->get_parameter("ref_scale").as_double();

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
      torque_limit_.size() != num_joints_ || torque_rate_limit_.size() != num_joints_)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "参数维度不匹配: joints=%zu, kp=%zu, kd=%zu, limit=%zu, rate=%zu",
      num_joints_, default_kp_.size(), default_kd_.size(),
      torque_limit_.size(), torque_rate_limit_.size());
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

  // 每关节一个 effort 命令接口
  for (const auto & joint : joint_names_)
  {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
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
    const double kp = reference_interfaces_[ref_offsets_[i].kp];
    const double kd = reference_interfaces_[ref_offsets_[i].kd];

    // Step 3: 阻抗控制律
    // τ = Kp * (q_ref - q) + Kd * (q̇_ref - q̇) + τ_ff
    double tau_target = 0.0;
    if (!zero_torque_)
    {
      const double pos_error = q_ref - q_current;
      const double vel_error = qdot_ref - qdot_filt_[i];
      tau_target = kp * pos_error + kd * vel_error + tau_ff;
    }

    // 变化率限制 (含超时归零: 平滑下降到 0)
    const double max_delta = torque_rate_limit_[i] * dt;
    double delta = tau_target - tau_prev_[i];
    delta = std::clamp(delta, -max_delta, max_delta);
    double tau = tau_prev_[i] + delta;

    // 力矩限幅
    tau = std::clamp(tau, -torque_limit_[i], torque_limit_[i]);

    tau_prev_[i] = tau;

    // 写入命令接口
    command_interfaces_[i].set_value(tau);
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

  return controller_interface::return_type::OK;
}

}  // namespace exoskeleton_controllers

// ===================== Pluginlib 注册 =====================

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  exoskeleton_controllers::ExoImpedanceController,
  controller_interface::ChainableControllerInterface)
