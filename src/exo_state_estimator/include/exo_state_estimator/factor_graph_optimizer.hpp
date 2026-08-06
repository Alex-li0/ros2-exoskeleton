#pragma once

#include <Eigen/Dense>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "exo_state_estimator/contact_detector.hpp"
#include "exo_state_estimator/fk_solver.hpp"

namespace exo_state_estimator {

struct EstimatorConfig {
  // ISAM2 参数
  int relinearize_skip = 1;
  double relinearize_threshold = 0.01;

  // 滑动窗口
  double lag_seconds = 5.0;
  double frame_dt = 0.005;  // 200Hz

  // 噪声参数
  gtsam::Vector6 prior_sigmas;
  gtsam::Vector6 vio_sigmas;
  double terrain_sigma_sensor = 0.01;
  Eigen::Matrix3d kinematic_cov_stick;
  double kinematic_w_valid = 0.999;

  // 地形参数
  double terrain_height = 0.0;
  Eigen::Vector3d terrain_normal{0, 0, 1};

  // FK
  std::string urdf_path;

  // IMU 参数 (独立 IMU 传感器, 非 VIO 内置)
  bool use_imu_factor = true;
  double imu_gravity = 9.81;
  double imu_accel_noise = 0.01;      // 加速度计噪声密度 (m/s^2/sqrt(Hz))
  double imu_gyro_noise = 0.001;      // 陀螺仪噪声密度 (rad/s/sqrt(Hz))
  double imu_accel_bias_rw = 0.0002;  // 加速度计偏置随机游走
  double imu_gyro_bias_rw = 0.00003;  // 陀螺仪偏置随机游走
  Eigen::Matrix3d imu_body_P_sensor;  // IMU 相对于盆腔的外参旋转

  EstimatorConfig() {
    prior_sigmas << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;
    vio_sigmas << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1;
    kinematic_cov_stick = Eigen::Vector3d(0.0025, 0.0025, 0.0025).asDiagonal();
    imu_body_P_sensor = Eigen::Matrix3d::Identity();
  }
};

struct SensorData {
  double timestamp = 0.0;

  // 独立 IMU 传感器 (非 VIO 内置)
  Eigen::Vector3d accel{0, 0, 0};   // 线加速度 (m/s^2)
  Eigen::Vector3d gyro{0, 0, 0};    // 角速度 (rad/s)
  bool imu_valid = false;

  // VIO (来自 OpenVINS)
  gtsam::Pose3 vio_pose;
  Eigen::Matrix<double, 6, 6> vio_covariance = Eigen::Matrix<double, 6, 6>::Identity();
  bool vio_valid = false;

  // 关节状态 (6个主动关节: [left_hip3, left_knee, left_ankle, right_hip3, right_knee, right_ankle])
  std::vector<double> joint_positions;
  std::vector<double> joint_velocities;

  // 高程图查询结果
  double terrain_z = 0.0;
  Eigen::Vector3d terrain_normal{0, 0, 1};
  double terrain_sigma = 0.01;
  bool terrain_valid = false;

  // 接触状态
  ContactState contact_state = ContactState::SWING;

  // CPG 状态 (辅助)
  std::string cpg_state;
};

struct EstimatorOutput {
  double timestamp = 0.0;
  gtsam::Pose3 pose;
  gtsam::Vector3 velocity{0, 0, 0};           // 盆腔速度 (body frame)
  gtsam::Vector3 accel_bias{0, 0, 0};         // 加速度计偏置
  gtsam::Vector3 gyro_bias{0, 0, 0};          // 陀螺仪偏置
  Eigen::Matrix<double, 6, 6> pose_cov = Eigen::Matrix<double, 6, 6>::Identity();
  ContactState contact_state = ContactState::SWING;

  // 诊断信息
  int num_factors = 0;
  double optimize_time_ms = 0.0;
  bool slip_detected_left = false;
  bool slip_detected_right = false;
};

class FactorGraphOptimizer {
 public:
  bool init(const EstimatorConfig& config);
  void processSensorData(const SensorData& data);
  bool getLatestOutput(EstimatorOutput& output) const;
  void reset();

 private:
  EstimatorConfig config_;
  std::unique_ptr<gtsam::ISAM2> isam_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_estimate_;
  FKSolver fk_solver_;

  // 状态管理
  int frame_count_ = 0;
  std::deque<int> window_frames_;
  std::set<int> inserted_landmarks_;
  int landmark_counter_ = 0;
  bool initialized_ = false;

  // IMU 预积分
  std::unique_ptr<gtsam::PreintegratedImuMeasurements> imu_preintegrated_;
  gtsam::NavState prev_state_;
  gtsam::imuBias::ConstantBias prev_bias_;
  double prev_imu_timestamp_ = 0.0;
  bool imu_initialized_ = false;

  // 上一帧 VIO
  gtsam::Pose3 prev_vio_pose_;
  bool prev_vio_valid_ = false;

  // 输出缓冲
  mutable std::mutex output_mutex_;
  EstimatorOutput latest_output_;
  bool has_output_ = false;

  // 当前帧时间戳
  double current_timestamp_ = 0.0;

  // 上一帧优化结果
  gtsam::Pose3 last_optimized_pose_;
  gtsam::Vector3 last_optimized_velocity_{0, 0, 0};

  // 内部方法
  void initializeFirstFrame(const SensorData& data);
  void addVIOFactor(const SensorData& data);
  void addIMUFactor(const SensorData& data);
  void addKinematicFactors(const SensorData& data);
  void runOptimization();
  void updateOutput(const SensorData& data);
  void marginalizeOldFrames();

  gtsam::Key poseKey(int frame) const { return gtsam::symbol_shorthand::X(frame); }
  gtsam::Key velKey(int frame) const { return gtsam::symbol_shorthand::V(frame); }
  gtsam::Key landmarkKey(int id) const { return gtsam::symbol_shorthand::L(id); }
  gtsam::Key biasKey(int frame) const { return gtsam::symbol_shorthand::B(frame); }
};

}  // namespace exo_state_estimator
