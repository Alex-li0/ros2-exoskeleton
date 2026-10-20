#include "exo_state_estimator/factor_graph_optimizer.hpp"

#include <chrono>
#include <cmath>
#include <iostream>

#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include "exo_state_estimator/kinematic_contact_factor.hpp"
#include "exo_state_estimator/terrain_height_factor.hpp"

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::L;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

namespace exo_state_estimator {

bool FactorGraphOptimizer::init(const EstimatorConfig& config) {
  config_ = config;
  if (!(config_.lag_seconds > 0) || !(config_.max_imu_gap > 0) ||
      !config_.prior_sigmas.allFinite() || (config_.prior_sigmas.array() <= 0).any() ||
      !config_.vio_sigmas.allFinite() || (config_.vio_sigmas.array() <= 0).any()) return false;
  for (double value : {config_.lag_seconds, config_.max_imu_gap, config_.imu_gravity,
      config_.imu_accel_noise, config_.imu_gyro_noise,
      config_.imu_accel_bias_rw, config_.imu_gyro_bias_rw}) {
    if (!std::isfinite(value) || value <= 0) return false;
  }
  if (!config_.left_contact_offset.allFinite() || !config_.right_contact_offset.allFinite() ||
      !config_.body_P_imu.matrix().allFinite()) return false;
  fk_ready_ = !config_.urdf_path.empty() && fk_solver_.initFromFile(config_.urdf_path);
  if (!config_.urdf_path.empty() && !fk_ready_) return false;
  if (config_.contact_geometry_calibrated && !fk_ready_) return false;
  if (config_.use_imu_factor) {
    auto params = gtsam::PreintegrationParams::MakeSharedU(config_.imu_gravity);
    params->accelerometerCovariance = Eigen::Matrix3d::Identity() * std::pow(config_.imu_accel_noise, 2);
    params->gyroscopeCovariance = Eigen::Matrix3d::Identity() * std::pow(config_.imu_gyro_noise, 2);
    params->integrationCovariance = Eigen::Matrix3d::Identity() * 1e-8;
    params->setBodyPSensor(config_.body_P_imu);
    imu_preintegrated_ = std::make_unique<gtsam::PreintegratedImuMeasurements>(
        params, gtsam::imuBias::ConstantBias());
  }
  reset();
  return true;
}

bool FactorGraphOptimizer::processSensorData(const SensorData& data) {
  if (!data.vio_valid || !std::isfinite(data.timestamp) || data.timestamp < 0 ||
      (initialized_ && data.timestamp <= current_timestamp_) ||
      !data.vio_pose.matrix().allFinite()) return false;
  if (!initialized_) {
    initializeFirstFrame(data);
    return true;
  }
  if (config_.use_imu_factor && !integrateIMU(data)) return false;
  if (config_.use_imu_factor) addIMUFactor();
  graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(X(frame_count_ - 1), X(frame_count_),
      prev_vio_pose_.between(data.vio_pose),
      gtsam::noiseModel::Diagonal::Sigmas(config_.vio_sigmas)));
  if (!config_.use_imu_factor) {
    initial_estimate_.insert(X(frame_count_), data.vio_pose);
  }
  timestamps_[X(frame_count_)] = data.timestamp;
  addKinematicFactors(data);
  runOptimization(data);
  prev_vio_pose_ = data.vio_pose;
  ++frame_count_;
  return true;
}

void FactorGraphOptimizer::initializeFirstFrame(const SensorData& data) {
  graph_.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), data.vio_pose,
      gtsam::noiseModel::Diagonal::Sigmas(config_.prior_sigmas)));
  initial_estimate_.insert(X(0), data.vio_pose);
  timestamps_[X(0)] = data.timestamp;
  prev_bias_ = gtsam::imuBias::ConstantBias();
  prev_state_ = gtsam::NavState(data.vio_pose, gtsam::Vector3::Zero());
  if (config_.use_imu_factor) {
    // Startup requires a stationary interval. Bias must be connected from frame zero.
    graph_.add(gtsam::PriorFactor<gtsam::Vector3>(V(0), gtsam::Vector3::Zero(),
        gtsam::noiseModel::Isotropic::Sigma(3, 0.1)));
    gtsam::Vector6 bias_sigmas;
    bias_sigmas << .1, .1, .1, .01, .01, .01;
    graph_.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(0), prev_bias_,
        gtsam::noiseModel::Diagonal::Sigmas(bias_sigmas)));
    initial_estimate_.insert(V(0), prev_state_.velocity());
    initial_estimate_.insert(B(0), prev_bias_);
    timestamps_[V(0)] = data.timestamp;
    timestamps_[B(0)] = data.timestamp;
  }
  addKinematicFactors(data);
  runOptimization(data);
  prev_vio_pose_ = data.vio_pose;
  initialized_ = true;
  frame_count_ = 1;
}

bool FactorGraphOptimizer::integrateIMU(const SensorData& data) {
  const auto& samples = data.imu_samples;
  if (samples.size() < 2 || samples.front().timestamp > current_timestamp_ ||
      samples.back().timestamp < data.timestamp) return false;
  for (size_t i = 0; i < samples.size(); ++i) {
    if (!std::isfinite(samples[i].timestamp) || !samples[i].accel.allFinite() ||
        !samples[i].gyro.allFinite() ||
        (i > 0 && samples[i].timestamp <= samples[i - 1].timestamp)) return false;
  }
  imu_preintegrated_->resetIntegrationAndSetBias(prev_bias_);
  double covered = current_timestamp_;
  for (size_t i = 0; i + 1 < samples.size(); ++i) {
    const double begin = std::max(current_timestamp_, samples[i].timestamp);
    const double end = std::min(data.timestamp, samples[i + 1].timestamp);
    if (end <= begin) continue;
    if (samples[i + 1].timestamp - samples[i].timestamp > config_.max_imu_gap ||
        begin > covered + 1e-9) return false;
    imu_preintegrated_->integrateMeasurement(samples[i].accel, samples[i].gyro, end - begin);
    covered = end;
  }
  return std::abs(covered - data.timestamp) < 1e-8;
}

void FactorGraphOptimizer::addIMUFactor() {
  graph_.add(gtsam::ImuFactor(X(frame_count_ - 1), V(frame_count_ - 1),
      X(frame_count_), V(frame_count_), B(frame_count_ - 1), *imu_preintegrated_));
  gtsam::Vector6 sigmas;
  const double root_dt = std::sqrt(imu_preintegrated_->deltaTij());
  sigmas << config_.imu_accel_bias_rw, config_.imu_accel_bias_rw, config_.imu_accel_bias_rw,
      config_.imu_gyro_bias_rw, config_.imu_gyro_bias_rw, config_.imu_gyro_bias_rw;
  sigmas *= root_dt;
  graph_.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(frame_count_ - 1),
      B(frame_count_), gtsam::imuBias::ConstantBias(),
      gtsam::noiseModel::Diagonal::Sigmas(sigmas)));
  const auto prediction = imu_preintegrated_->predict(prev_state_, prev_bias_);
  initial_estimate_.insert(X(frame_count_), prediction.pose());
  initial_estimate_.insert(V(frame_count_), prediction.velocity());
  initial_estimate_.insert(B(frame_count_), prev_bias_);
  // Current keyframe timestamp is assigned by processSensorData.
}

bool FactorGraphOptimizer::footPositions(const std::vector<double>& joints,
    const gtsam::Pose3& pose, Eigen::Vector3d& left, Eigen::Vector3d& right) const {
  if (!fk_ready_ || !config_.contact_geometry_calibrated || joints.size() != 6) return false;
  auto lf = fk_solver_.compute("left", {joints.begin(), joints.begin() + 3}, config_.left_contact_offset);
  auto rf = fk_solver_.compute("right", {joints.begin() + 3, joints.end()}, config_.right_contact_offset);
  if (!lf.valid || !rf.valid) return false;
  left = pose.transformFrom(lf.p_foot);
  right = pose.transformFrom(rf.p_foot);
  return true;
}

void FactorGraphOptimizer::addKinematicFactors(const SensorData& data) {
  const bool geometry_valid = fk_ready_ && config_.contact_geometry_calibrated &&
      data.joint_positions.size() == 6;
  auto add = [&](const char* leg, int begin, const FootSupport& support,
                 const Eigen::Vector3d& offset, const TerrainObservation& terrain,
                 int& landmark, uint64_t& session) {
    if (!geometry_valid || !support.valid || !support.stable) {
      landmark = -1;
      return;
    }
    std::vector<double> joints(data.joint_positions.begin() + begin,
                               data.joint_positions.begin() + begin + 3);
    const auto fk = fk_solver_.compute(leg, joints, offset);
    if (!fk.valid) { landmark = -1; return; }
    if (landmark < 0 || session != support.session) {
      landmark = landmark_counter_++;
      session = support.session;
      const auto prediction = initial_estimate_.at<gtsam::Pose3>(X(frame_count_));
      initial_estimate_.insert(L(landmark), prediction.transformFrom(fk.p_foot));
      // One terrain observation per new contact; never invent a z=0 plane.
      if (config_.use_terrain_factor && terrain.valid && terrain.point.allFinite() &&
          terrain.normal.allFinite() && terrain.normal.norm() > 1e-6 &&
          std::isfinite(terrain.sigma) && terrain.sigma > 0) {
        graph_.add(TerrainHeightLandmarkFactor(L(landmark), terrain.point,
            terrain.normal, terrain.sigma));
      }
    }
    graph_.add(KinematicContactFactor(X(frame_count_), L(landmark), fk.p_foot,
        gtsam::noiseModel::Gaussian::Covariance(config_.kinematic_cov_stick)));
    // Keep an active shared landmark in the fixed-lag window.
    timestamps_[L(landmark)] = data.timestamp;
  };
  add("left", 0, data.support.left, config_.left_contact_offset, data.left_terrain,
      left_landmark_, left_session_);
  add("right", 3, data.support.right, config_.right_contact_offset, data.right_terrain,
      right_landmark_, right_session_);
}

void FactorGraphOptimizer::runOptimization(const SensorData& data) {
  const auto start = std::chrono::steady_clock::now();
  if (config_.use_imu_factor) {
    timestamps_[V(frame_count_)] = data.timestamp;
    timestamps_[B(frame_count_)] = data.timestamp;
  }
  smoother_->update(graph_, initial_estimate_, timestamps_);
  graph_.resize(0);
  initial_estimate_.clear();
  timestamps_.clear();
  const auto result = smoother_->calculateEstimate();
  EstimatorOutput output;
  output.timestamp = data.timestamp;
  output.pose = result.at<gtsam::Pose3>(X(frame_count_));
  output.pose_cov = smoother_->marginalCovariance(X(frame_count_));
  if (config_.use_imu_factor) {
    output.velocity = result.at<gtsam::Vector3>(V(frame_count_));
    output.velocity_cov = smoother_->marginalCovariance(V(frame_count_));
    prev_bias_ = result.at<gtsam::imuBias::ConstantBias>(B(frame_count_));
    output.accel_bias = prev_bias_.accelerometer();
    output.gyro_bias = prev_bias_.gyroscope();
  } else if (initialized_) {
    output.velocity = (output.pose.translation() - prev_state_.position()) /
        (data.timestamp - current_timestamp_);
  }
  prev_state_ = gtsam::NavState(output.pose, output.velocity);
  current_timestamp_ = data.timestamp;
  output.contact_state = data.support.state();
  output.stable_support = data.support.stableState();
  output.left_landmark = left_landmark_;
  output.right_landmark = right_landmark_;
  output.num_states = static_cast<int>(result.size());
  for (const auto& factor : smoother_->getFactors()) if (factor) ++output.num_factors;
  output.optimize_time_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
  std::lock_guard<std::mutex> lock(output_mutex_);
  latest_output_ = output;
  has_output_ = true;
}

bool FactorGraphOptimizer::getLatestOutput(EstimatorOutput& output) const {
  std::lock_guard<std::mutex> lock(output_mutex_);
  if (!has_output_) return false;
  output = latest_output_;
  return true;
}

void FactorGraphOptimizer::reset() {
  gtsam::ISAM2Params params;
  params.relinearizeSkip = config_.relinearize_skip;
  params.relinearizeThreshold = config_.relinearize_threshold;
  params.findUnusedFactorSlots = true;
  smoother_ = std::make_unique<gtsam::IncrementalFixedLagSmoother>(config_.lag_seconds, params);
  graph_.resize(0);
  initial_estimate_.clear();
  timestamps_.clear();
  frame_count_ = 0;
  landmark_counter_ = 0;
  left_landmark_ = right_landmark_ = -1;
  left_session_ = right_session_ = 0;
  initialized_ = false;
  current_timestamp_ = -1.0;
  if (imu_preintegrated_) {
    imu_preintegrated_->resetIntegrationAndSetBias(gtsam::imuBias::ConstantBias());
  }
  std::lock_guard<std::mutex> lock(output_mutex_);
  has_output_ = false;
}

}  // namespace exo_state_estimator
