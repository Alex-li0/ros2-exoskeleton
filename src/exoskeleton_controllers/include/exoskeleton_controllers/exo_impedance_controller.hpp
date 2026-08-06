#ifndef EXOSKELETON_CONTROLLERS__EXO_IMPEDANCE_CONTROLLER_HPP_
#define EXOSKELETON_CONTROLLERS__EXO_IMPEDANCE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
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
  rclcpp::Time last_ref_time_;
  bool ref_data_valid_{false};
  // 参考数据超时/未收到时置位 → 输出零力矩 (安全默认)
  bool zero_torque_{false};

  // 调试发布器 (200Hz): /impedance/debug (JointState: position=q_ref, velocity=q_measured, effort=tau)
  //                    /impedance/timing (Float64MultiArray: [loop_dt, ref_age_s])
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr debug_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr timing_pub_;

  // 参考接口偏移量索引
  std::vector<ReferenceOffsets> ref_offsets_;
};

}  // namespace exoskeleton_controllers

#endif  // EXOSKELETON_CONTROLLERS__EXO_IMPEDANCE_CONTROLLER_HPP_
