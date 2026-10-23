#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <exo_state_estimator/msg/gait_event.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "cpg_controller/adaptive_cpg.hpp"
#include "cpg_controller/learning_interface.hpp"
#include "cpg_controller/reference_math.hpp"

using namespace std::chrono_literals;

// Deliberately independent of legs_cpg and all production controller topics.
class AdaptiveCPGShadowNode : public rclcpp::Node
{
public:
  AdaptiveCPGShadowNode() : Node("adaptive_cpg_shadow")
  {
    enabled_ = declare_parameter("enable_learning", false);
    cpg_controller::AdaptiveConfig cfg;
    cfg.omega_initial = declare_parameter("omega_initial", cfg.omega_initial);
    cfg.omega_min = declare_parameter("omega_min", cfg.omega_min);
    cfg.omega_max = declare_parameter("omega_max", cfg.omega_max);
    cfg.phase_gain = declare_parameter("phase_gain", cfg.phase_gain);
    cfg.frequency_gain = declare_parameter("frequency_gain", cfg.frequency_gain);
    cfg.shape_gain = declare_parameter("shape_gain", cfg.shape_gain);
    timeout_ = cfg.measurement_timeout = declare_parameter("measurement_timeout", cfg.measurement_timeout);
    cfg.shape_enabled = declare_parameter("shape_enabled", true);
    cfg.shape_requires_lock = declare_parameter("shape_requires_lock", true);
    const bool events_enabled = declare_parameter("events_enabled", false);
    cfg.events_enabled = enabled_ && events_enabled;
    event_phases_ = declare_parameter("event_phases", std::vector<double>{0.0, std::acos(-1.0)});
    if (event_phases_.size() != 2 || !std::isfinite(event_phases_[0]) || !std::isfinite(event_phases_[1]))
      throw std::runtime_error("event_phases must contain finite LEFT/RIGHT phase targets");

    std::array<cpg_controller::Coefficients, 6> prior;
    for (size_t j = 0; j < names_.size(); ++j) {
      const auto prefix = "joint." + names_[j] + ".";
      prior[j].offset.value = declare_parameter(prefix + "w_offset", 0.0);
      const auto x = declare_parameter(prefix + "w_x", std::vector<double>{0, 0, 0, 0});
      const auto y = declare_parameter(prefix + "w_y", std::vector<double>{0, 0, 0, 0});
      limits_low_[j] = declare_parameter(prefix + "limit_min", -1.5);
      limits_high_[j] = declare_parameter(prefix + "limit_max", 1.5);
      if (x.size() != y.size() || x.size() < 2 || x.size() > 4 ||
          !std::isfinite(prior[j].offset.value) ||
          !std::isfinite(limits_low_[j]) || !std::isfinite(limits_high_[j]) ||
          limits_low_[j] >= limits_high_[j])
        throw std::runtime_error("Invalid shadow joint template: " + names_[j]);
      for (size_t n = 0; n < x.size(); ++n) {
        if (!std::isfinite(x[n]) || !std::isfinite(y[n]))
          throw std::runtime_error("Nonfinite shadow coefficients: " + names_[j]);
        if (n > 0) { prior[j].x[n].value = x[n]; prior[j].y[n].value = y[n]; }
      }
    }
    core_ = std::make_unique<cpg_controller::AdaptiveCPG>(prior, cfg, now().seconds());
    reference_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_ref_shadow", 10);
    debug_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("learning_debug", 10);
    measurement_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "measured_joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        ++received_;
        if (!enabled_ || msg->header.stamp.sec < 0 || msg->header.stamp.nanosec >= 1000000000u)
          return;
        measurements_.accept(msg->name, msg->position, rclcpp::Time(msg->header.stamp).seconds(),
          now().seconds(), timeout_, names_);
      });
    if (std::string(measurement_sub_->get_topic_name()) == reference_pub_->get_topic_name())
      throw std::runtime_error("Measurement input must not alias shadow output");
    if (cfg.events_enabled) {
      event_sub_ = create_subscription<exo_state_estimator::msg::GaitEvent>(
        "gait_events", rclcpp::SensorDataQoS(),
        [this](exo_state_estimator::msg::GaitEvent::ConstSharedPtr msg) {
          using Event = exo_state_estimator::msg::GaitEvent;
          ++events_received_;
          if (!msg->valid || msg->event != Event::INITIAL_CONTACT ||
              (msg->foot_id != Event::LEFT && msg->foot_id != Event::RIGHT) ||
              !std::isfinite(msg->confidence) || msg->confidence < .7 ||
              msg->header.stamp.sec < 0 || msg->header.stamp.nanosec >= 1000000000u ||
              pending_count_ == pending_.size()) return;
          pending_[pending_count_++] = {rclcpp::Time(msg->header.stamp).seconds(),
            static_cast<size_t>(msg->foot_id - Event::LEFT), msg->event_id, msg->confidence};
        });
    }
    timer_ = create_wall_timer(2ms, std::bind(&AdaptiveCPGShadowNode::tick, this));
    RCLCPP_INFO(get_logger(), "Shadow CPG ready: learning=%s, events=%s; reference=%s",
      enabled_ ? "enabled" : "disabled", cfg.events_enabled ? "enabled" : "disabled",
      reference_pub_->get_topic_name());
  }

private:
  void tick()
  {
    const auto stamp = now();
    const double time = stamp.seconds();
    core_->step(time);
    if (enabled_ && measurements_.frame().stamp > consumed_stamp_) {
      const auto &measurement = measurements_.frame();
      consumed_stamp_ = measurement.stamp;  // consume an accepted timestamp once only
      if (core_->observe(measurement, time)) ++accepted_;
    }
    for (size_t i = 0; i < pending_count_; ++i) {
      const auto &event = pending_[i];
      if (core_->event(event.stamp, event.foot, event.id, event.confidence,
          event_phases_[event.foot], time)) ++events_accepted_;
    }
    pending_count_ = 0;
    const auto output = core_->output();
    sensor_msgs::msg::JointState reference;
    reference.header.stamp = stamp;
    reference.name = names_;
    std::array<double, 6> clipped{};
    for (size_t j = 0; j < names_.size(); ++j) {
      const auto limited = cpg_controller::constrain({output.position[j], output.velocity[j]},
        {limits_low_[j], 0}, {limits_high_[j], 0});
      reference.position.push_back(limited.value);
      reference.velocity.push_back(limited.rate);
      clipped[j] = output.position[j] <= limits_low_[j] || output.position[j] >= limits_high_[j] ? 1.0 : 0.0;
    }
    reference_pub_->publish(reference);

    // Fixed layout, 74 doubles: [0] stamp, [1] enable_learning,
    // [2] state (WAITING=0,CAPTURE=1,LOCKED=2,LOST=3), [3] unwrapped phase,
    // [4] base omega, [5] actual phase rate, [6] local phase error,
    // [7] residual RMS rad, [8] measurement age s, [9] shape active,
    // [10:14] received/accepted measurements, received/accepted events,
    // [14:20] raw reconstruction q, [20:26] raw reconstruction qdot,
    // [26:32] output clamp flags. [32:74] six sets of
    // offset,x1,y1,x2,y2,x3,y3 (same joint order as JointState).
    std_msgs::msg::Float64MultiArray debug;
    debug.data = {time, enabled_ ? 1.0 : 0.0, static_cast<double>(output.state),
      output.phase, output.omega_base, output.phase_rate, output.phase_error,
      output.residual_rms, output.measurement_age, output.shape_active ? 1.0 : 0.0,
      static_cast<double>(received_), static_cast<double>(accepted_),
      static_cast<double>(events_received_), static_cast<double>(events_accepted_)};
    debug.data.insert(debug.data.end(), output.position.begin(), output.position.end());
    debug.data.insert(debug.data.end(), output.velocity.begin(), output.velocity.end());
    debug.data.insert(debug.data.end(), clipped.begin(), clipped.end());
    for (const auto &joint : core_->weights()) {
      debug.data.push_back(joint.offset.value);
      for (size_t n = 1; n <= 3; ++n) {
        debug.data.push_back(joint.x[n].value);
        debug.data.push_back(joint.y[n].value);
      }
    }
    debug_pub_->publish(debug);
  }

  struct PendingEvent { double stamp; size_t foot; uint64_t id; double confidence; };
  const std::vector<std::string> names_{"left_hip3", "left_knee", "left_ankle",
    "right_hip3", "right_knee", "right_ankle"};
  bool enabled_{false};
  double timeout_{.1}, consumed_stamp_{0};
  std::array<double, 6> limits_low_{}, limits_high_{};
  std::vector<double> event_phases_;
  uint64_t received_{0}, accepted_{0}, events_received_{0}, events_accepted_{0};
  std::array<PendingEvent, 16> pending_{};
  size_t pending_count_{0};
  cpg_controller::MeasurementInput measurements_;
  std::unique_ptr<cpg_controller::AdaptiveCPG> core_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr reference_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr measurement_sub_;
  rclcpp::Subscription<exo_state_estimator::msg::GaitEvent>::SharedPtr event_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try { rclcpp::spin(std::make_shared<AdaptiveCPGShadowNode>()); }
  catch (const std::exception &error) { std::cerr << error.what() << std::endl; result = 1; }
  rclcpp::shutdown();
  return result;
}
