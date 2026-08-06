#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace exo_state_estimator {

enum class ContactState : uint8_t {
  SWING = 0,        // 摆动
  RIGHT_STANCE = 1, // 右脚支撑
  LEFT_STANCE = 2,  // 左脚支撑
  DOUBLE_STANCE = 3 // 双脚支撑
};

struct ContactDetectionInput {
  // 小腿 IMU 加速度 (可选)
  double accel_shank_left_z = 0.0;
  double accel_shank_right_z = 0.0;
  // FK 计算的足端速度 (可选)
  double foot_vel_left_z = 0.0;
  double foot_vel_right_z = 0.0;
  // CPG 状态 (可选，可用于辅助判断)
  std::string cpg_state;
  // 关节力矩 (可选)
  std::vector<double> joint_torques;
};

struct ContactDetectionOutput {
  ContactState state;
  double confidence_right;  // 右脚支撑置信度 [0,1]
  double confidence_left;   // 左脚支撑置信度 [0,1]
};

// 接触检测器接口 (抽象基类)
// 具体实现方式由用户决定：阈值法、学习法、IMU+FK融合法等
class ContactDetector {
 public:
  virtual ~ContactDetector() = default;

  virtual void reset() = 0;
  virtual ContactDetectionOutput detect(const ContactDetectionInput& input) = 0;
  virtual std::string getName() const = 0;
};

// 简单阈值检测器 (默认实现，用于测试)
class ThresholdContactDetector : public ContactDetector {
 public:
  struct Config {
    double accel_threshold = 2.0;   // m/s^2, 超过此值认为可能触地
    double vel_threshold = 0.05;    // m/s, 低于此值认为静止
    double hysteresis = 0.1;        // 迟滞系数
    int min_stance_frames = 3;      // 最少连续支撑帧数
  };

  explicit ThresholdContactDetector(const Config& config);

  void reset() override;
  ContactDetectionOutput detect(const ContactDetectionInput& input) override;
  std::string getName() const override { return "threshold"; }

 private:
  Config config_;
  ContactState prev_state_ = ContactState::SWING;
  int right_stance_count_ = 0;
  int left_stance_count_ = 0;
};

}  // namespace exo_state_estimator
