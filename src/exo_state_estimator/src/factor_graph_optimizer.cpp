#include "exo_state_estimator/factor_graph_optimizer.hpp"

#include <chrono>
#include <iostream>

#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include "exo_state_estimator/kinematic_max_mixture_factor.hpp"
#include "exo_state_estimator/terrain_height_factor.hpp"

using gtsam::symbol_shorthand::B;  // bias
using gtsam::symbol_shorthand::L;  // landmark
using gtsam::symbol_shorthand::V;  // velocity
using gtsam::symbol_shorthand::X;  // pose

namespace exo_state_estimator {

bool FactorGraphOptimizer::init(const EstimatorConfig& config) {
  config_ = config;

  gtsam::ISAM2Params isam_params;
  isam_params.relinearizeSkip = config_.relinearize_skip;
  isam_params.relinearizeThreshold = config_.relinearize_threshold;
  isam_ = std::make_unique<gtsam::ISAM2>(isam_params);

  // FK 求解器
  if (!config_.urdf_path.empty()) {
    if (!fk_solver_.initFromFile(config_.urdf_path)) {
      std::cerr << "[Optimizer] Failed to load URDF: " << config_.urdf_path
                << std::endl;
      return false;
    }
  } else {
    std::cerr << "[Optimizer] WARNING: No URDF path, FK disabled" << std::endl;
  }

  // IMU 预积分器初始化
  if (config_.use_imu_factor) {
    auto imu_params = gtsam::PreintegrationParams::MakeSharedU(config_.imu_gravity);
    imu_params->accelerometerCovariance =
        Eigen::Matrix3d::Identity() * std::pow(config_.imu_accel_noise, 2);
    imu_params->gyroscopeCovariance =
        Eigen::Matrix3d::Identity() * std::pow(config_.imu_gyro_noise, 2);
    imu_params->integrationCovariance =
        Eigen::Matrix3d::Identity() * 1e-8;

    // 外参: IMU 相对于盆腔的旋转
    Eigen::Matrix3d body_P_sensor = config_.imu_body_P_sensor;
    imu_params->setBodyPSensor(
        gtsam::Pose3(gtsam::Rot3(body_P_sensor), gtsam::Point3::Zero()));

    imu_preintegrated_ = std::make_unique<gtsam::PreintegratedImuMeasurements>(
        imu_params, gtsam::imuBias::ConstantBias());

    std::cout << "[Optimizer] IMU factor enabled (standalone sensor)" << std::endl;
  }

  frame_count_ = 0;
  landmark_counter_ = 0;
  initialized_ = false;
  imu_initialized_ = false;
  prev_vio_valid_ = false;
  prev_imu_timestamp_ = 0.0;

  std::cout << "[Optimizer] Initialized" << std::endl;
  return true;
}

void FactorGraphOptimizer::processSensorData(const SensorData& data) {
  // IMU 预积分累积 (独立 IMU 传感器, 高频 200-400Hz)
  if (config_.use_imu_factor && data.imu_valid && imu_initialized_ &&
      prev_imu_timestamp_ > 0) {
    double dt = data.timestamp - prev_imu_timestamp_;
    if (dt > 0 && dt < 0.1) {
      imu_preintegrated_->integrateMeasurement(data.accel, data.gyro, dt);
    }
  }

  // 首帧初始化 (需要 VIO 给出初始位姿)
  if (!initialized_) {
    if (data.vio_valid) {
      initializeFirstFrame(data);
      prev_imu_timestamp_ = data.timestamp;
      imu_initialized_ = true;
    }
    return;
  }

  // IMU 因子 (必须在 VIO 之前, 因为它提供帧间状态传播)
  if (config_.use_imu_factor) {
    addIMUFactor(data);
  }

  // VIO 因子 (帧间相对位姿, 来自 OpenVINS)
  addVIOFactor(data);

  // 运动学因子 (Max-Mixture + 高程图)
  addKinematicFactors(data);

  // 优化
  current_timestamp_ = data.timestamp;
  runOptimization();

  // 输出
  updateOutput(data);

  // 滑动窗口
  marginalizeOldFrames();

  prev_imu_timestamp_ = data.timestamp;
  frame_count_++;
}

void FactorGraphOptimizer::initializeFirstFrame(const SensorData& data) {
  gtsam::Pose3 X0 = data.vio_pose;
  gtsam::Vector3 V0 = gtsam::Vector3::Zero();
  gtsam::imuBias::ConstantBias B0;

  // 位姿先验
  auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(config_.prior_sigmas);
  graph_.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), X0, prior_noise));

  // 初始猜测
  initial_estimate_.insert(X(0), X0);

  if (config_.use_imu_factor) {
    // 速度先验 (初始速度为零)
    auto vel_prior_noise =
        gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
    graph_.add(gtsam::PriorFactor<gtsam::Vector3>(V(0), V0, vel_prior_noise));

    initial_estimate_.insert(V(0), V0);
    initial_estimate_.insert(B(0), B0);

    prev_state_ = gtsam::NavState(X0, V0);
    prev_bias_ = B0;
  }

  isam_->update(graph_, initial_estimate_);
  graph_.resize(0);
  initial_estimate_.clear();

  last_optimized_pose_ = X0;
  last_optimized_velocity_ = V0;
  prev_vio_pose_ = X0;
  prev_vio_valid_ = true;
  initialized_ = true;
  window_frames_.push_back(0);

  std::cout << "[Optimizer] First frame at t=" << data.timestamp << std::endl;
}

void FactorGraphOptimizer::addVIOFactor(const SensorData& data) {
  if (!data.vio_valid || !prev_vio_valid_) return;

  gtsam::Pose3 delta_vio = prev_vio_pose_.between(data.vio_pose);

  // 噪声: 从 VIO 协方差对角线提取, 或使用配置的固定值
  gtsam::Vector6 vio_sigma;
  bool use_cov_diag = (data.vio_covariance(0, 0) > 1e-12);
  if (use_cov_diag) {
    for (int i = 0; i < 6; ++i) {
      vio_sigma(i) = std::sqrt(std::max(1e-10, data.vio_covariance(i, i)));
    }
  } else {
    vio_sigma = config_.vio_sigmas;
  }
  auto vio_noise = gtsam::noiseModel::Diagonal::Sigmas(vio_sigma);

  graph_.add(
      gtsam::BetweenFactor<gtsam::Pose3>(X(frame_count_ - 1), X(frame_count_),
                                          delta_vio, vio_noise));

  // 用 VIO 绝对位姿作为当前帧初始猜测 (如果 IMU 没有提供)
  if (!config_.use_imu_factor && !initial_estimate_.exists(X(frame_count_))) {
    initial_estimate_.insert(X(frame_count_), data.vio_pose);
  }

  prev_vio_pose_ = data.vio_pose;
}

void FactorGraphOptimizer::addIMUFactor(const SensorData& data) {
  if (!data.imu_valid) return;

  // 添加 IMU 因子: 约束 X(i-1), V(i-1) -> X(i), V(i), B(i-1)
  gtsam::PreintegratedImuMeasurements preint_imu =
      *dynamic_cast<gtsam::PreintegratedImuMeasurements*>(
          imu_preintegrated_.get());

  graph_.add(gtsam::ImuFactor(
      X(frame_count_ - 1), V(frame_count_ - 1), X(frame_count_),
      V(frame_count_), B(frame_count_ - 1), preint_imu));

  // Bias 随机游走因子: B(i-1) -> B(i)
  auto bias_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector6() << config_.imu_accel_bias_rw, config_.imu_accel_bias_rw,
       config_.imu_accel_bias_rw, config_.imu_gyro_bias_rw,
       config_.imu_gyro_bias_rw, config_.imu_gyro_bias_rw)
          .finished());
  graph_.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
      B(frame_count_ - 1), B(frame_count_),
      gtsam::imuBias::ConstantBias(), bias_noise));

  // 用 IMU 预积分预测当前状态作为初始猜测
  gtsam::NavState predicted = preint_imu.predict(prev_state_, prev_bias_);
  if (!initial_estimate_.exists(X(frame_count_))) {
    initial_estimate_.insert(X(frame_count_), predicted.pose());
  }
  if (!initial_estimate_.exists(V(frame_count_))) {
    initial_estimate_.insert(V(frame_count_), predicted.velocity());
  }
  if (!initial_estimate_.exists(B(frame_count_))) {
    initial_estimate_.insert(B(frame_count_), prev_bias_);
  }

  // 重置预积分器 (以当前偏置为基准)
  imu_preintegrated_->resetIntegrationAndSetBias(prev_bias_);
}

void FactorGraphOptimizer::addKinematicFactors(const SensorData& data) {
  if (data.joint_positions.size() < 6) return;

  bool use_right = (data.contact_state == ContactState::RIGHT_STANCE ||
                    data.contact_state == ContactState::DOUBLE_STANCE);
  bool use_left = (data.contact_state == ContactState::LEFT_STANCE ||
                   data.contact_state == ContactState::DOUBLE_STANCE);

  // 右腿: joints [3,4,5] = right_hip3, right_knee, right_ankle
  if (use_right) {
    std::vector<double> right_joints(data.joint_positions.begin() + 3,
                                     data.joint_positions.begin() + 6);
    FKResult fk = fk_solver_.compute("right", right_joints);
    if (fk.valid) {
      int lid = landmark_counter_++;
      gtsam::Key lk = L(lid);

      graph_.add(KinematicMaxMixtureFactor(X(frame_count_), lk, fk.p_foot,
                                           config_.kinematic_cov_stick,
                                           config_.kinematic_w_valid));

      double z_t = data.terrain_valid ? data.terrain_z : config_.terrain_height;
      Eigen::Vector3d n_t =
          data.terrain_valid ? data.terrain_normal : config_.terrain_normal;
      double s_t =
          data.terrain_valid ? data.terrain_sigma : config_.terrain_sigma_sensor;
      graph_.add(TerrainHeightLandmarkFactor(lk, z_t, n_t, s_t));

      gtsam::Pose3 X_pred = initial_estimate_.exists(X(frame_count_))
                                 ? initial_estimate_.at<gtsam::Pose3>(X(frame_count_))
                                 : last_optimized_pose_;
      gtsam::Point3 L_init = X_pred.transformFrom(fk.p_foot);
      if (!inserted_landmarks_.count(lid)) {
        initial_estimate_.insert(lk, L_init);
        inserted_landmarks_.insert(lid);
      }
    }
  }

  // 左腿: joints [0,1,2] = left_hip3, left_knee, left_ankle
  if (use_left) {
    std::vector<double> left_joints(data.joint_positions.begin(),
                                    data.joint_positions.begin() + 3);
    FKResult fk = fk_solver_.compute("left", left_joints);
    if (fk.valid) {
      int lid = landmark_counter_++;
      gtsam::Key lk = L(lid);

      graph_.add(KinematicMaxMixtureFactor(X(frame_count_), lk, fk.p_foot,
                                           config_.kinematic_cov_stick,
                                           config_.kinematic_w_valid));

      double z_t = data.terrain_valid ? data.terrain_z : config_.terrain_height;
      Eigen::Vector3d n_t =
          data.terrain_valid ? data.terrain_normal : config_.terrain_normal;
      double s_t =
          data.terrain_valid ? data.terrain_sigma : config_.terrain_sigma_sensor;
      graph_.add(TerrainHeightLandmarkFactor(lk, z_t, n_t, s_t));

      gtsam::Pose3 X_pred = initial_estimate_.exists(X(frame_count_))
                                 ? initial_estimate_.at<gtsam::Pose3>(X(frame_count_))
                                 : last_optimized_pose_;
      gtsam::Point3 L_init = X_pred.transformFrom(fk.p_foot);
      if (!inserted_landmarks_.count(lid)) {
        initial_estimate_.insert(lk, L_init);
        inserted_landmarks_.insert(lid);
      }
    }
  }
}

void FactorGraphOptimizer::runOptimization() {
  auto t_start = std::chrono::high_resolution_clock::now();

  isam_->update(graph_, initial_estimate_);
  graph_.resize(0);
  initial_estimate_.clear();

  gtsam::Values result = isam_->calculateBestEstimate();

  last_optimized_pose_ = result.at<gtsam::Pose3>(X(frame_count_));

  if (config_.use_imu_factor) {
    last_optimized_velocity_ = result.at<gtsam::Vector3>(V(frame_count_));
    prev_state_ = gtsam::NavState(last_optimized_pose_, last_optimized_velocity_);
    prev_bias_ = result.at<gtsam::imuBias::ConstantBias>(B(frame_count_));
  }

  auto t_end = std::chrono::high_resolution_clock::now();
  double opt_time =
      std::chrono::duration<double, std::milli>(t_end - t_start).count();

  bool slip_left = false, slip_right = false;

  std::lock_guard<std::mutex> lock(output_mutex_);
  latest_output_.timestamp = current_timestamp_;
  latest_output_.pose = last_optimized_pose_;
  latest_output_.optimize_time_ms = opt_time;
  latest_output_.num_factors =
      static_cast<int>(isam_->getFactorsUnsafe().size());
  latest_output_.slip_detected_left = slip_left;
  latest_output_.slip_detected_right = slip_right;
  has_output_ = true;

  window_frames_.push_back(frame_count_);
}

void FactorGraphOptimizer::updateOutput(const SensorData& data) {
  std::lock_guard<std::mutex> lock(output_mutex_);
  latest_output_.contact_state = data.contact_state;

  if (config_.use_imu_factor) {
    latest_output_.velocity = last_optimized_velocity_;
    latest_output_.accel_bias = prev_bias_.accelerometer();
    latest_output_.gyro_bias = prev_bias_.gyroscope();
  }

  try {
    gtsam::Matrix cov = isam_->marginalCovariance(X(frame_count_));
    latest_output_.pose_cov = cov.block<6, 6>(0, 0);
  } catch (...) {
    latest_output_.pose_cov = Eigen::Matrix<double, 6, 6>::Identity() * 0.01;
  }
}

void FactorGraphOptimizer::marginalizeOldFrames() {
  int window_size = static_cast<int>(config_.lag_seconds / config_.frame_dt);
  while (static_cast<int>(window_frames_.size()) > window_size) {
    window_frames_.pop_front();
  }
}

bool FactorGraphOptimizer::getLatestOutput(EstimatorOutput& output) const {
  std::lock_guard<std::mutex> lock(output_mutex_);
  if (!has_output_) return false;
  output = latest_output_;
  return true;
}

void FactorGraphOptimizer::reset() {
  frame_count_ = 0;
  landmark_counter_ = 0;
  initialized_ = false;
  imu_initialized_ = false;
  prev_vio_valid_ = false;
  prev_imu_timestamp_ = 0.0;
  window_frames_.clear();
  inserted_landmarks_.clear();

  gtsam::ISAM2Params isam_params;
  isam_params.relinearizeSkip = config_.relinearize_skip;
  isam_ = std::make_unique<gtsam::ISAM2>(isam_params);

  graph_ = gtsam::NonlinearFactorGraph();
  initial_estimate_ = gtsam::Values();

  if (config_.use_imu_factor && imu_preintegrated_) {
    imu_preintegrated_->resetIntegrationAndSetBias(gtsam::imuBias::ConstantBias());
  }

  has_output_ = false;
  std::cout << "[Optimizer] Reset" << std::endl;
}

}  // namespace exo_state_estimator
