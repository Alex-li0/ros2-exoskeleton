#ifndef EXOSKELETON_CONTROLLERS__EXO_IMPEDANCE_CONTROLLER_HPP_
#define EXOSKELETON_CONTROLLERS__EXO_IMPEDANCE_CONTROLLER_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace exoskeleton_controllers
{

// 每关节参考接口布局
struct ReferenceOffsets
{
  size_t target_position;
  size_t target_velocity;
  size_t feedforward_torque;
  size_t kp;
  size_t kd;
};

struct TuningCommand
{
  bool session_active{false};
  uint64_t enabled_mask{0};
  std::vector<double> kp;
  std::vector<double> kd;
  std::vector<double> tube_half_width;
  std::vector<double> tube_transition_width;
  uint64_t zero_sequence{0};
  int64_t stamp_ns{0};
};

struct AuxiliaryCommand
{
  bool received{false};
  bool transparency_enabled{false};
  bool assist_enabled{false};
  std::vector<double> transparency_iq;
  std::vector<double> assist_iq;
  int64_t stamp_ns{0};
};

class ExoImpedanceController : public controller_interface::ChainableControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  // 导出参考接口（供上游控制器链式写入）
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  // 切换链式模式（禁用/启用订阅者）
  bool on_set_chained_mode(bool chained_mode) override;

  // 非链式模式下，从订阅者更新参考值
  controller_interface::return_type update_reference_from_subscribers() override;

  // 核心控制计算
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // 参数
  std::vector<std::string> joint_names_;
  std::vector<double> default_kp_;
  std::vector<double> default_kd_;
  std::vector<double> default_tube_half_width_;
  std::vector<double> default_tube_transition_width_;
  std::vector<double> torque_limit_;
  std::vector<double> torque_rate_limit_;
  double velocity_filter_alpha_{0.2};
  double data_timeout_{0.01};
  double ref_scale_{1.0};  // 参考缩放 (0~1): 调试时降低跟踪幅值, 避免力矩饱和, 不改 CPG 参数

  size_t num_joints_{0};

  // 内部状态（用于滤波和变化率限制）
  std::vector<double> qdot_filt_;
  std::vector<double> tau_prev_;

  // 订阅者（仅非链式模式使用）
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr ref_sub_;
  // V3调参命令: [3, enabled_mask, zero_sequence, kp0, kd0, tube0, transition0, ...]。
  // 进入调参会话后超时即全关节归零。
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr tuning_sub_;
  // 未来实时补偿叠加量:
  // [1, transparency_enabled, assist_enabled, transparency_iq[N], assist_iq[N]]。
  // 没有发布者、未启用或超时均严格按零处理。
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr auxiliary_sub_;
  rclcpp::Time last_ref_time_;
  bool ref_data_valid_{false};
  // 参考数据超时/未收到时置位 → 输出零力矩 (安全默认)
  bool zero_torque_{false};
  realtime_tools::RealtimeBuffer<TuningCommand> tuning_command_buffer_;
  realtime_tools::RealtimeBuffer<AuxiliaryCommand> auxiliary_command_buffer_;
  uint64_t applied_zero_sequence_{0};
  double tuning_command_timeout_{0.25};
  double auxiliary_command_timeout_{0.1};

  // 调试发布器 (200Hz): /impedance/debug (JointState: position=q_ref, velocity=q_measured, effort=tau)
  //                    /impedance/timing (Float64MultiArray: [loop_dt, ref_age_s])
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr debug_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr timing_pub_;
  // [tube_iq[N], transparency_iq[N], assist_iq[N], actual_command_iq[N]]
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr component_pub_;

  // 参考接口偏移量索引
  std::vector<ReferenceOffsets> ref_offsets_;
};

}  // namespace exoskeleton_controllers

#endif  // EXOSKELETON_CONTROLLERS__EXO_IMPEDANCE_CONTROLLER_HPP_
