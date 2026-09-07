#include <chrono>
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "terrain_adaptive_cpg/msg/cpg_debug.hpp"
#include "terrain_adaptive_cpg/msg/cpg_parameter_target.hpp"
#include "terrain_adaptive_cpg/msg/terrain_modulation.hpp"
#include "terrain_adaptive_cpg/msg/terrain_scheduler_debug.hpp"
#include "terrain_adaptive_cpg/scheduler_core.hpp"

using namespace std::chrono_literals;

namespace terrain_adaptive_cpg
{

class TerrainParameterScheduler : public rclcpp::Node
{
public:
  TerrainParameterScheduler() : Node("terrain_parameter_scheduler")
  {
    mode_switching_enabled_ = declare_parameter("mode_switching_enabled", false);
    continuous_enabled_ = declare_parameter("continuous_modulation_enabled", false);
    simulation_only_enabled_ = declare_parameter("simulation_only_enabled", false);
    continuous_model_type_ = declare_parameter("continuous_model_type", std::string("disabled"));
    terrain_timeout_s_ = declare_parameter("terrain_input_timeout_s", 0.0);
    odometry_timeout_s_ = declare_parameter("odometry_timeout_s", 0.0);
    contact_timeout_s_ = declare_parameter("contact_timeout_s", 0.0);
    minimum_confidence_ = declare_parameter("minimum_confidence", -1.0);
    enforce_covariance_limit_ = declare_parameter("enforce_covariance_limit", false);
    max_pose_covariance_diagonal_ = declare_parameter("max_pose_covariance_diagonal", 0.0);
    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names", {"left_hip3", "left_knee", "left_ankle", "right_hip3", "right_knee", "right_ankle"});
    verified_gaits_ = declare_parameter<std::vector<std::string>>("verified_gaits", {"flat_walk"});

    const auto terrain_names = declare_parameter<std::vector<std::string>>(
      "terrain_mode_names", {"flat"});
    const auto gait_names = declare_parameter<std::vector<std::string>>(
      "terrain_mode_gaits", {"flat_walk"});
    if (terrain_names.size() != gait_names.size()) {
      throw std::runtime_error("terrain_mode_names and terrain_mode_gaits size mismatch");
    }
    for (size_t i = 0; i < terrain_names.size(); ++i) terrain_to_gait_[terrain_names[i]] = gait_names[i];

    limits_.omega_scale_min = declare_parameter("limits.omega_scale_min", 1.0);
    limits_.omega_scale_max = declare_parameter("limits.omega_scale_max", 1.0);
    limits_.amplitude_scale_min = declare_parameter("limits.amplitude_scale_min", 1.0);
    limits_.amplitude_scale_max = declare_parameter("limits.amplitude_scale_max", 1.0);
    limits_.offset_delta_min = declare_parameter("limits.offset_delta_min", 0.0);
    limits_.offset_delta_max = declare_parameter("limits.offset_delta_max", 0.0);
    limits_.omega_scale_rate = declare_parameter("limits.omega_scale_rate", 0.0);
    limits_.amplitude_scale_rate = declare_parameter("limits.amplitude_scale_rate", 0.0);
    limits_.offset_delta_rate = declare_parameter("limits.offset_delta_rate", 0.0);
    limits_.filter_time_constant = declare_parameter("limits.filter_time_constant", 0.0);

    simulation_request_.omega_scale = declare_parameter("simulation_target.omega_scale", 1.0);
    const double amplitude = declare_parameter("simulation_target.amplitude_scale", 1.0);
    const double offset = declare_parameter("simulation_target.offset_delta", 0.0);
    simulation_request_.amplitude_scale.assign(joint_names_.size(), amplitude);
    simulation_request_.offset_delta.assign(joint_names_.size(), offset);
    current_target_.amplitude_scale.assign(joint_names_.size(), 1.0);
    current_target_.offset_delta.assign(joint_names_.size(), 0.0);

    terrain_sub_ = create_subscription<msg::TerrainModulation>(
      "/terrain/modulation", 10,
      [this](msg::TerrainModulation::SharedPtr message) {
        terrain_ = *message;
        terrain_receive_time_ = now();
        terrain_received_ = true;
      });
    command_sub_ = create_subscription<std_msgs::msg::String>(
      "/cpg/gait_mode_cmd", 10,
      [this](std_msgs::msg::String::SharedPtr message) { pending_manual_mode_ = message->data; });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/fused/odometry", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr message) {
        odom_ = *message;
        odom_receive_time_ = now();
        odom_received_ = true;
      });
    contact_sub_ = create_subscription<std_msgs::msg::UInt8>(
      "/fused/contact_state", 10,
      [this](std_msgs::msg::UInt8::SharedPtr message) { onContact(message->data); });
    cpg_debug_sub_ = create_subscription<msg::CPGDebug>(
      "/cpg/debug", 10,
      [this](msg::CPGDebug::SharedPtr message) {
        cpg_phase_ = message->phase;
        cpg_phase_valid_ = std::isfinite(message->phase);
        cpg_state_ = message->state;
        active_gait_ = message->gait_mode;
      });

    gait_target_pub_ = create_publisher<std_msgs::msg::String>("/cpg/gait_mode_target", 10);
    parameter_target_pub_ = create_publisher<msg::CPGParameterTarget>("/cpg/parameter_target", 10);
    debug_pub_ = create_publisher<msg::TerrainSchedulerDebug>("/terrain/scheduler_debug", 10);
    timer_ = create_wall_timer(20ms, std::bind(&TerrainParameterScheduler::tick, this));
    last_tick_ = now();

    if (!mode_switching_enabled_ || minimum_confidence_ < 0.0 || terrain_timeout_s_ <= 0.0 ||
      odometry_timeout_s_ <= 0.0 || contact_timeout_s_ <= 0.0)
    {
      RCLCPP_WARN(get_logger(),
        "生产安全门限尚未标定，模式切换保持关闭。TODO(data_required): 设置输入超时、置信度和状态估计门限。");
    }
    if (!continuous_enabled_) {
      RCLCPP_INFO(get_logger(), "连续参数调制关闭；仅保持已验证 gait library 参数");
    }
    if (continuous_model_type_ != "disabled") {
      RCLCPP_WARN(get_logger(),
        "连续模型 '%s' 仅有接口、没有标定数据；生产路径不会生成参数",
        continuous_model_type_.c_str());
    }
  }

private:
  bool fresh(const rclcpp::Time & receive_time, double timeout) const
  {
    return timeout > 0.0 && (now() - receive_time).seconds() <= timeout;
  }

  bool isVerified(const std::string & gait) const
  {
    return std::find(verified_gaits_.begin(), verified_gaits_.end(), gait) != verified_gaits_.end();
  }

  bool odometryValid() const
  {
    if (!odom_received_ || !fresh(odom_receive_time_, odometry_timeout_s_)) return false;
    const rclcpp::Time source_stamp(odom_.header.stamp);
    const double source_age = (now() - source_stamp).seconds();
    if (source_age < 0.0 || source_age > odometry_timeout_s_) return false;
    const auto & p = odom_.pose.pose.position;
    const auto & q = odom_.pose.pose.orientation;
    const auto & v = odom_.twist.twist.linear;
    const double q_norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z) ||
      !std::isfinite(q_norm) || q_norm < 1e-12)
    {
      return false;
    }
    std::vector<double> covariance(odom_.pose.covariance.begin(), odom_.pose.covariance.end());
    if (!covarianceStructurallyValid(covariance)) return false;
    if (enforce_covariance_limit_) {
      if (max_pose_covariance_diagonal_ <= 0.0) return false;
      for (size_t i = 0; i < 6; ++i) {
        if (covariance[i * 6 + i] > max_pose_covariance_diagonal_) return false;
      }
    }
    return true;
  }

  bool terrainFieldsValid() const
  {
    if (!terrain_.terrain_mode_valid || terrain_.terrain_mode.empty()) return false;
    const rclcpp::Time source_stamp(terrain_.stamp);
    const double source_age = (now() - source_stamp).seconds();
    if (source_age < 0.0 || source_age > terrain_timeout_s_) return false;
    const auto valid_number = [](bool valid, double value) {return !valid || std::isfinite(value);};
    return valid_number(terrain_.slope_valid, terrain_.slope) &&
      valid_number(terrain_.step_height_valid, terrain_.step_height) &&
      valid_number(terrain_.step_depth_valid, terrain_.step_depth) &&
      valid_number(terrain_.boundary_distance_valid, terrain_.boundary_distance) &&
      valid_number(terrain_.confidence_valid, terrain_.confidence);
  }

  void onContact(uint8_t state)
  {
    const bool previous_left = contact_state_ == 2 || contact_state_ == 3;
    const bool previous_right = contact_state_ == 1 || contact_state_ == 3;
    const bool current_left = state == 2 || state == 3;
    const bool current_right = state == 1 || state == 3;
    if (!previous_left && current_left && cpg_phase_valid_) {
      left_contact_phase_ = cpg_phase_;
      left_contact_phase_valid_ = true;
    }
    if (!previous_right && current_right && cpg_phase_valid_) {
      right_contact_phase_ = cpg_phase_;
      right_contact_phase_valid_ = true;
    }
    contact_state_ = state;
    contact_receive_time_ = now();
    contact_received_ = state <= 3;
  }

  std::string contactLabel() const
  {
    switch (contact_state_) {
      case 0: return "swing";
      case 1: return "right_stance";
      case 2: return "left_stance";
      case 3: return "double_stance";
      default: return "invalid";
    }
  }

  void publishParameterTarget(bool enable, bool simulation)
  {
    msg::CPGParameterTarget output;
    output.stamp = now();
    output.gait_mode = active_gait_;
    output.gait_mode_valid = !active_gait_.empty();
    output.continuous_modulation_enabled = enable;
    output.omega_scale = current_target_.omega_scale;
    output.omega_scale_valid = enable;
    // TODO(data_required): 接触相位统计足够后才允许非零相位校正。
    output.phase_rate_correction = 0.0;
    output.phase_rate_correction_valid = false;
    output.joint_names = joint_names_;
    output.amplitude_scale = current_target_.amplitude_scale;
    output.amplitude_scale_valid.assign(joint_names_.size(), enable);
    output.offset_delta = current_target_.offset_delta;
    output.offset_delta_valid.assign(joint_names_.size(), enable);
    output.simulation_only = simulation;
    parameter_target_pub_->publish(output);
  }

  void tick()
  {
    const auto stamp = now();
    const double dt = std::max(0.0, (stamp - last_tick_).seconds());
    last_tick_ = stamp;
    const bool terrain_fresh = terrain_received_ && fresh(terrain_receive_time_, terrain_timeout_s_);
    const bool fields_valid = terrain_fresh && terrainFieldsValid();
    const bool confidence_ok = fields_valid && terrain_.confidence_valid &&
      minimum_confidence_ >= 0.0 && terrain_.confidence >= minimum_confidence_;
    const bool odom_valid = odometryValid();
    const bool contact_valid = contact_received_ && fresh(contact_receive_time_, contact_timeout_s_);
    const bool state_window = cpg_state_ == "WALKING" && contact_valid && contactAllowsSwitch(contact_state_);
    const bool switch_allowed = mode_switching_enabled_ && fields_valid && confidence_ok && odom_valid && state_window;

    std::string requested = pending_manual_mode_;
    if (requested.empty() && fields_valid) {
      const auto it = terrain_to_gait_.find(terrain_.terrain_mode);
      if (it != terrain_to_gait_.end()) requested = it->second;
    }

    std::string reason = "hold_verified_parameters";
    if (!terrain_fresh) reason = terrain_received_ ? "terrain_timeout" : "terrain_not_received";
    else if (!fields_valid) reason = "invalid_terrain_fields";
    else if (!confidence_ok) reason = "confidence_rejected_or_unconfigured";
    else if (!odom_valid) reason = "odometry_invalid_or_stale";
    else if (!contact_valid) reason = "contact_invalid_or_stale";
    else if (!state_window) reason = "waiting_for_walking_double_stance";
    else if (!mode_switching_enabled_) reason = "mode_switching_disabled_data_required";

    if (switch_allowed && !requested.empty() && requested != active_gait_) {
      const bool simulation_exception = simulation_only_enabled_ && terrain_.simulation_only;
      if (isVerified(requested) || simulation_exception) {
        std_msgs::msg::String target;
        target.data = requested;
        gait_target_pub_->publish(target);
        pending_manual_mode_.clear();
        reason = simulation_exception && !isVerified(requested) ?
          "simulation_only_unverified_mode_target" : "verified_mode_target_sent";
      } else {
        reason = "unverified_gait_rejected";
      }
    }

    const bool continuous_actionable = continuous_enabled_ && fields_valid && confidence_ok && odom_valid &&
      contact_valid && (!terrain_.simulation_only || simulation_only_enabled_);
    if (continuous_actionable && terrain_.simulation_only) {
      current_target_ = applyLimits(current_target_, simulation_request_, limits_, dt);
      publishParameterTarget(true, true);
    } else {
      current_target_.omega_scale = 1.0;
      std::fill(current_target_.amplitude_scale.begin(), current_target_.amplitude_scale.end(), 1.0);
      std::fill(current_target_.offset_delta.begin(), current_target_.offset_delta.end(), 0.0);
      publishParameterTarget(false, false);
    }

    msg::TerrainSchedulerDebug debug;
    debug.header.stamp = stamp;
    debug.terrain_received = terrain_received_;
    debug.terrain_timed_out = terrain_received_ && !terrain_fresh;
    debug.terrain_fields_valid = fields_valid;
    debug.confidence_accepted = confidence_ok;
    debug.odometry_valid = odom_valid;
    debug.contact_valid = contact_valid;
    debug.contact_state = contact_state_;
    debug.contact_label = contactLabel();
    debug.switch_allowed = switch_allowed;
    debug.modulation_frozen = !continuous_actionable;
    debug.fallback_active = !fields_valid || !confidence_ok || !odom_valid || !contact_valid;
    debug.active_gait_mode = active_gait_;
    debug.requested_gait_mode = requested;
    debug.decision_reason = reason;
    debug.cpg_phase = cpg_phase_;
    debug.cpg_phase_valid = cpg_phase_valid_;
    debug.left_contact_phase = left_contact_phase_;
    debug.left_contact_phase_valid = left_contact_phase_valid_;
    debug.right_contact_phase = right_contact_phase_;
    debug.right_contact_phase_valid = right_contact_phase_valid_;
    debug_pub_->publish(debug);
  }

  bool mode_switching_enabled_{false}, continuous_enabled_{false}, simulation_only_enabled_{false};
  bool terrain_received_{false}, odom_received_{false}, contact_received_{false};
  bool cpg_phase_valid_{false}, left_contact_phase_valid_{false}, right_contact_phase_valid_{false};
  bool enforce_covariance_limit_{false};
  double terrain_timeout_s_{0.0}, odometry_timeout_s_{0.0}, contact_timeout_s_{0.0};
  double minimum_confidence_{-1.0}, max_pose_covariance_diagonal_{0.0};
  double cpg_phase_{0.0}, left_contact_phase_{0.0}, right_contact_phase_{0.0};
  uint8_t contact_state_{255};
  std::string cpg_state_, active_gait_, pending_manual_mode_, continuous_model_type_;
  std::vector<std::string> joint_names_, verified_gaits_;
  std::map<std::string, std::string> terrain_to_gait_;
  msg::TerrainModulation terrain_;
  nav_msgs::msg::Odometry odom_;
  ContinuousLimits limits_;
  ContinuousTarget current_target_, simulation_request_;
  rclcpp::Time terrain_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time odom_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time contact_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_tick_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<msg::TerrainModulation>::SharedPtr terrain_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr contact_sub_;
  rclcpp::Subscription<msg::CPGDebug>::SharedPtr cpg_debug_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gait_target_pub_;
  rclcpp::Publisher<msg::CPGParameterTarget>::SharedPtr parameter_target_pub_;
  rclcpp::Publisher<msg::TerrainSchedulerDebug>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace terrain_adaptive_cpg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<terrain_adaptive_cpg::TerrainParameterScheduler>());
  rclcpp::shutdown();
  return 0;
}
