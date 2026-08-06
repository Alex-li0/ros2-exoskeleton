#include "exo_state_estimator/contact_detector.hpp"

#include <algorithm>
#include <cmath>

namespace exo_state_estimator {

ThresholdContactDetector::ThresholdContactDetector(const Config& config)
    : config_(config) {}

void ThresholdContactDetector::reset() {
  prev_state_ = ContactState::SWING;
  right_stance_count_ = 0;
  left_stance_count_ = 0;
}

ContactDetectionOutput ThresholdContactDetector::detect(
    const ContactDetectionInput& input) {
  ContactDetectionOutput output;

  // 基于加速度和速度的简单阈值判断
  bool right_contact =
      std::abs(input.accel_shank_right_z) > config_.accel_threshold ||
      std::abs(input.foot_vel_right_z) < config_.vel_threshold;

  bool left_contact =
      std::abs(input.accel_shank_left_z) > config_.accel_threshold ||
      std::abs(input.foot_vel_left_z) < config_.vel_threshold;

  // 迟滞: 如果之前是支撑态, 需要更强的信号才切换到摆动
  if (prev_state_ == ContactState::RIGHT_STANCE ||
      prev_state_ == ContactState::DOUBLE_STANCE) {
    if (!right_contact) {
      // 需要信号低于阈值 * (1 - hysteresis) 才切换
      right_contact =
          std::abs(input.accel_shank_right_z) >
              config_.accel_threshold * (1.0 - config_.hysteresis) ||
          std::abs(input.foot_vel_right_z) <
              config_.vel_threshold * (1.0 + config_.hysteresis);
    }
  }
  if (prev_state_ == ContactState::LEFT_STANCE ||
      prev_state_ == ContactState::DOUBLE_STANCE) {
    if (!left_contact) {
      left_contact =
          std::abs(input.accel_shank_left_z) >
              config_.accel_threshold * (1.0 - config_.hysteresis) ||
          std::abs(input.foot_vel_left_z) <
              config_.vel_threshold * (1.0 + config_.hysteresis);
    }
  }

  // 连续帧计数
  if (right_contact)
    right_stance_count_++;
  else
    right_stance_count_ = 0;

  if (left_contact)
    left_stance_count_++;
  else
    left_stance_count_ = 0;

  // 只有连续帧数达标才算真正触地
  bool right_stable = right_stance_count_ >= config_.min_stance_frames;
  bool left_stable = left_stance_count_ >= config_.min_stance_frames;

  // 确定接触状态
  if (right_stable && left_stable) {
    output.state = ContactState::DOUBLE_STANCE;
  } else if (right_stable) {
    output.state = ContactState::RIGHT_STANCE;
  } else if (left_stable) {
    output.state = ContactState::LEFT_STANCE;
  } else {
    output.state = ContactState::SWING;
  }

  output.confidence_right =
      std::min(1.0, static_cast<double>(right_stance_count_) /
                        config_.min_stance_frames);
  output.confidence_left =
      std::min(1.0, static_cast<double>(left_stance_count_) /
                        config_.min_stance_frames);

  prev_state_ = output.state;
  return output;
}

}  // namespace exo_state_estimator
