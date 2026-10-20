#pragma once

#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include "exo_state_estimator/contact_detector.hpp"
#include "exo_state_estimator/fk_solver.hpp"

namespace exo_state_estimator {

struct ImuSample {
  double timestamp = 0.0;
  Eigen::Vector3d accel = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
};

struct TerrainObservation {
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal{0, 0, 1};
  double sigma = 0.01;
  bool valid = false;
};

struct EstimatorConfig {
  int relinearize_skip = 1;
  double relinearize_threshold = 0.01;
  double lag_seconds = 5.0;
  gtsam::Vector6 prior_sigmas = gtsam::Vector6::Constant(0.01);
  // Empirical relative-pose noise, [rotation, translation]; not absolute VIO covariance.
  gtsam::Vector6 vio_sigmas = (gtsam::Vector6() << .1, .1, .1, .05, .05, .05).finished();
  Eigen::Matrix3d kinematic_cov_stick = .0025 * Eigen::Matrix3d::Identity();
  std::string urdf_path;
  bool contact_geometry_calibrated = false;
  Eigen::Vector3d left_contact_offset = Eigen::Vector3d::Zero();
  Eigen::Vector3d right_contact_offset = Eigen::Vector3d::Zero();
  bool use_terrain_factor = false;
  bool use_imu_factor = true;
  double imu_gravity = 9.81;
  double imu_accel_noise = 0.01;
  double imu_gyro_noise = 0.001;
  double imu_accel_bias_rw = 0.0002;
  double imu_gyro_bias_rw = 0.00003;
  double max_imu_gap = 0.02;
  gtsam::Pose3 body_P_imu;
};

struct SensorData {
  double timestamp = 0.0;  // VIO/keyframe measurement time.
  // Brackets the entire previous-keyframe -> current-keyframe interval.
  std::vector<ImuSample> imu_samples;
  gtsam::Pose3 vio_pose;  // Must already describe the pelvis, not a moving leg IMU.
  bool vio_valid = false;
  std::vector<double> joint_positions;
  SupportState support;
  TerrainObservation left_terrain;
  TerrainObservation right_terrain;
};

struct EstimatorOutput {
  double timestamp = 0.0;
  gtsam::Pose3 pose;
  gtsam::Vector3 velocity = gtsam::Vector3::Zero();  // Navigation/world frame.
  gtsam::Vector3 accel_bias = gtsam::Vector3::Zero();
  gtsam::Vector3 gyro_bias = gtsam::Vector3::Zero();
  Eigen::Matrix<double, 6, 6> pose_cov = Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::Matrix3d velocity_cov = 1e6 * Eigen::Matrix3d::Identity();  // Unknown without IMU state.
  ContactState contact_state = ContactState::UNKNOWN;
  ContactState stable_support = ContactState::UNKNOWN;
  int num_factors = 0;
  int num_states = 0;
  int left_landmark = -1;
  int right_landmark = -1;
  double optimize_time_ms = 0.0;
};

class FactorGraphOptimizer {
 public:
  bool init(const EstimatorConfig& config);
  bool processSensorData(const SensorData& data);
  bool getLatestOutput(EstimatorOutput& output) const;
  bool footPositions(const std::vector<double>& joints, const gtsam::Pose3& pose,
                     Eigen::Vector3d& left, Eigen::Vector3d& right) const;
  void reset();

 private:
  EstimatorConfig config_;
  std::unique_ptr<gtsam::IncrementalFixedLagSmoother> smoother_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_estimate_;
  gtsam::FixedLagSmoother::KeyTimestampMap timestamps_;
  FKSolver fk_solver_;
  bool fk_ready_ = false;
  int frame_count_ = 0;
  int landmark_counter_ = 0;
  int left_landmark_ = -1;
  int right_landmark_ = -1;
  uint64_t left_session_ = 0;
  uint64_t right_session_ = 0;
  bool initialized_ = false;
  std::unique_ptr<gtsam::PreintegratedImuMeasurements> imu_preintegrated_;
  gtsam::NavState prev_state_;
  gtsam::imuBias::ConstantBias prev_bias_;
  gtsam::Pose3 prev_vio_pose_;
  double current_timestamp_ = -1.0;
  mutable std::mutex output_mutex_;
  EstimatorOutput latest_output_;
  bool has_output_ = false;

  void initializeFirstFrame(const SensorData& data);
  bool integrateIMU(const SensorData& data);
  void addIMUFactor();
  void addKinematicFactors(const SensorData& data);
  void runOptimization(const SensorData& data);
};

}  // namespace exo_state_estimator
