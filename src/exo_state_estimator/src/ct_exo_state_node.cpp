#include "exo_state_estimator/ct_exo_state_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

namespace exo_state_estimator {
namespace {
double seconds(const builtin_interfaces::msg::Time& stamp) {
  return rclcpp::Time(stamp).seconds();
}
builtin_interfaces::msg::Time messageTime(double stamp) {
  return rclcpp::Time(static_cast<int64_t>(std::llround(stamp * 1e9)));
}
Eigen::Vector3d vectorParameter(rclcpp::Node& node, const std::string& name) {
  const auto values = node.declare_parameter(name, std::vector<double>{0, 0, 0});
  if (values.size() != 3) throw std::invalid_argument(name + " must have three entries");
  Eigen::Vector3d result(values[0], values[1], values[2]);
  if (!result.allFinite()) throw std::invalid_argument(name + " must be finite");
  return result;
}
}  // namespace

CTExoStateNode::CTExoStateNode(const rclcpp::NodeOptions& options)
    : Node("ct_exo_state_node", options) {
  loadParameters();
  optimizer_ = std::make_unique<FactorGraphOptimizer>();
  if (!optimizer_->init(config_)) throw std::runtime_error("Invalid estimator configuration/URDF");
  contact_tracker_ = std::make_unique<GaitContactTracker>(
      declare_parameter("gait_state_timeout", .15), declare_parameter("gait_min_confidence", .8));
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      declare_parameter("imu_topic", "/imu/pelvis"), rclcpp::SensorDataQoS(),
      std::bind(&CTExoStateNode::imuCallback, this, std::placeholders::_1));
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      declare_parameter("joint_state_topic", "/joint_states"), rclcpp::SensorDataQoS(),
      std::bind(&CTExoStateNode::jointStateCallback, this, std::placeholders::_1));
  vio_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      declare_parameter("vio_topic", "/pelvis/vio_odometry"), rclcpp::SensorDataQoS(),
      std::bind(&CTExoStateNode::vioOdometryCallback, this, std::placeholders::_1));
  grid_map_sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
      "/elevation_mapping_node/elevation_map_raw", rclcpp::QoS(1).reliable(),
      std::bind(&CTExoStateNode::gridMapCallback, this, std::placeholders::_1));
  gait_event_sub_ = create_subscription<msg::GaitEvent>(
      "/gait/events", rclcpp::QoS(100).reliable(),
      std::bind(&CTExoStateNode::gaitEventCallback, this, std::placeholders::_1));
  gait_contact_sub_ = create_subscription<msg::GaitContactState>(
      "/gait/contact_state", rclcpp::QoS(100).reliable(),
      std::bind(&CTExoStateNode::gaitContactCallback, this, std::placeholders::_1));
  fused_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/fused/odometry", rclcpp::SensorDataQoS());
  contact_pub_ = create_publisher<std_msgs::msg::UInt8>("/fused/contact_state", 10);
  stable_support_pub_ = create_publisher<std_msgs::msg::UInt8>("/fused/stable_support", 10);
  diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/fused/diagnostics", 1);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  const double frequency = declare_parameter("optimize_frequency", 200.0);
  if (!std::isfinite(frequency) || frequency <= 0) throw std::invalid_argument("Invalid optimize_frequency");
  optimize_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / frequency),
      std::bind(&CTExoStateNode::optimizeTimerCallback, this));
  diagnostic_timer_ = create_wall_timer(std::chrono::milliseconds(100),
      std::bind(&CTExoStateNode::publishDiagnostics, this));
}

void CTExoStateNode::loadParameters() {
  world_frame_ = declare_parameter("world_frame", "global");
  pelvis_frame_ = declare_parameter("pelvis_frame", "pelvis_fused");
  vio_body_frame_ = declare_parameter("vio_body_frame", "base_link");
  config_.urdf_path = declare_parameter("urdf_path", std::string(""));
  config_.lag_seconds = declare_parameter("lag_seconds", 5.0);
  config_.use_imu_factor = declare_parameter("use_imu_factor", true);
  config_.use_terrain_factor = declare_parameter("use_terrain_factor", false);
  config_.contact_geometry_calibrated = declare_parameter("contact_geometry_calibrated", false);
  config_.left_contact_offset = vectorParameter(*this, "left_contact_offset");
  config_.right_contact_offset = vectorParameter(*this, "right_contact_offset");
  const auto xyz = vectorParameter(*this, "body_imu_xyz");
  const auto rpy = vectorParameter(*this, "body_imu_rpy");
  config_.body_P_imu = gtsam::Pose3(gtsam::Rot3::RzRyRx(rpy.x(), rpy.y(), rpy.z()), xyz);
  config_.imu_accel_noise = declare_parameter("imu_accel_noise", .01);
  config_.imu_gyro_noise = declare_parameter("imu_gyro_noise", .001);
  config_.imu_accel_bias_rw = declare_parameter("imu_accel_bias_rw", .0002);
  config_.imu_gyro_bias_rw = declare_parameter("imu_gyro_bias_rw", .00003);
  config_.max_imu_gap = declare_parameter("max_imu_gap", .02);
  const double pos = declare_parameter("prior_pos_sigma", .01);
  const double rot = declare_parameter("prior_rot_sigma", .01);
  config_.prior_sigmas << rot, rot, rot, pos, pos, pos;
  const double vp = declare_parameter("vio_pos_sigma", .05);
  const double vr = declare_parameter("vio_rot_sigma", .1);
  config_.vio_sigmas << vr, vr, vr, vp, vp, vp;
  const double kin = declare_parameter("kinematic_stick_sigma", .05);
  config_.kinematic_cov_stick = Eigen::Matrix3d::Identity() * kin * kin;
  const double keyframe_frequency = declare_parameter("keyframe_frequency", 25.0);
  if (!std::isfinite(keyframe_frequency) || keyframe_frequency <= 0) {
    throw std::invalid_argument("Invalid keyframe_frequency");
  }
  keyframe_period_ = 1.0 / keyframe_frequency;
  fusion_delay_ = declare_parameter("fusion_delay", .1);
  input_timeout_ = declare_parameter("input_timeout", .5);
  joint_max_gap_ = declare_parameter("joint_max_gap", .03);
  map_timeout_ = declare_parameter("map_timeout", .3);
  if (!std::isfinite(fusion_delay_) || !std::isfinite(input_timeout_) ||
      !std::isfinite(joint_max_gap_) || !std::isfinite(map_timeout_) ||
      fusion_delay_ < 0 || input_timeout_ <= fusion_delay_ ||
      joint_max_gap_ <= 0 || map_timeout_ <= 0 || kin <= 0) {
    throw std::invalid_argument("Invalid fusion timing/noise parameters");
  }
}

void CTExoStateNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  ImuSample sample;
  sample.timestamp = seconds(msg->header.stamp);
  sample.accel = {msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z};
  sample.gyro = {msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};
  if (!sample.accel.allFinite() || !sample.gyro.allFinite()) return;
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (!imu_samples_.empty() && sample.timestamp <= imu_samples_.back().timestamp) return;
  imu_samples_.push_back(sample);
  while (imu_samples_.size() > 8000) imu_samples_.pop_front();
}

void CTExoStateNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  JointSample sample{seconds(msg->header.stamp), {}};
  for (const auto& name : active_joint_names_) {
    const auto it = std::find(msg->name.begin(), msg->name.end(), name);
    if (it == msg->name.end()) return;
    const auto index = static_cast<size_t>(std::distance(msg->name.begin(), it));
    if (index >= msg->position.size() || !std::isfinite(msg->position[index])) return;
    sample.positions.push_back(msg->position[index]);
  }
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (!joint_samples_.empty() && sample.timestamp <= joint_samples_.back().timestamp) return;
  joint_samples_.push_back(std::move(sample));
  while (joint_samples_.size() > 8000) joint_samples_.pop_front();
}

void CTExoStateNode::vioOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  if (msg->header.frame_id != world_frame_ || msg->child_frame_id != vio_body_frame_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "VIO must already describe pelvis (%s -> %s); raw moving-leg odomimu is not accepted",
        world_frame_.c_str(), vio_body_frame_.c_str());
    return;
  }
  const auto& p = msg->pose.pose.position;
  const auto& q = msg->pose.pose.orientation;
  Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
  if (!Eigen::Vector3d(p.x, p.y, p.z).allFinite() || !rotation.coeffs().allFinite() ||
      std::abs(rotation.norm() - 1.0) > .01) return;
  SensorData data;
  data.timestamp = seconds(msg->header.stamp);
  data.vio_pose = gtsam::Pose3(gtsam::Rot3(rotation.normalized()), {p.x, p.y, p.z});
  data.vio_valid = true;
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (data.timestamp <= last_vio_stamp_) return;
  last_vio_stamp_ = data.timestamp;
  if (last_keyframe_stamp_ >= 0 && data.timestamp - last_keyframe_stamp_ < keyframe_period_ - 1e-8) return;
  last_keyframe_stamp_ = data.timestamp;
  vio_samples_.push_back(data);
  if (vio_samples_.size() > 200) {
    resetInputs();
    diagnostic_reason_ = "VIO queue overflow; reset, waiting for new observations";
  }
}

void CTExoStateNode::gaitEventCallback(const msg::GaitEvent::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  const double stamp = seconds(msg->header.stamp);
  const bool accepted = contact_tracker_->addEvent(stamp, msg->foot_id,
      static_cast<GaitEventType>(msg->event), msg->event_id, msg->valid, msg->confidence);
  if (!accepted && contact_tracker_->lastRejectionWasLate()) {
    // Old contact factors may be wrong: invalidate the whole local graph, not just the flag.
    optimizer_->reset();
    processed_stamp_ = -1;
    diagnostic_reason_ = "Late gait event: graph reset; increase validated fusion delay";
  }
}

void CTExoStateNode::gaitContactCallback(const msg::GaitContactState::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  const double stamp = seconds(msg->header.stamp);
  if (!contact_tracker_->addObservation(stamp, msg->foot_id, msg->any_contact,
      msg->full_foot_stable, msg->valid, msg->confidence) && contact_tracker_->lastRejectionWasLate()) {
    optimizer_->reset();
    processed_stamp_ = -1;
    diagnostic_reason_ = "Late contact observation: graph reset";
  }
}

void CTExoStateNode::gridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(map_mutex_);
  latest_grid_map_ = msg;
}

void CTExoStateNode::resetInputs() {
  imu_samples_.clear();
  joint_samples_.clear();
  vio_samples_.clear();
  contact_tracker_->reset();
  optimizer_->reset();
  processed_stamp_ = last_vio_stamp_ = last_keyframe_stamp_ = -1;
}

void CTExoStateNode::optimizeTimerCallback() {
  std::lock_guard<std::mutex> lock(data_mutex_);
  const double time = now().seconds();
  if (last_ros_time_ >= 0 && time < last_ros_time_) {
    resetInputs();
    std::lock_guard<std::mutex> map_lock(map_mutex_);
    latest_grid_map_.reset();
    diagnostic_reason_ = "ROS clock moved backwards; buffers and graph reset";
  }
  last_ros_time_ = time;
  for (int count = 0; count < 5 && !vio_samples_.empty(); ++count) {
    auto data = vio_samples_.front();
    const double age = time - data.timestamp;
    if (age < fusion_delay_) break;
    if (age > input_timeout_) {
      vio_samples_.pop_front();
      optimizer_->reset();
      processed_stamp_ = -1;
      diagnostic_reason_ = "Stale or unalignable input; waiting for a fresh interval";
      continue;
    }
    if (config_.use_imu_factor) {
      const double begin = processed_stamp_ >= 0 ? processed_stamp_ : data.timestamp;
      auto first = std::upper_bound(imu_samples_.begin(), imu_samples_.end(), begin,
          [](double t, const ImuSample& value) { return t < value.timestamp; });
      auto end = std::lower_bound(imu_samples_.begin(), imu_samples_.end(), data.timestamp,
          [](const ImuSample& value, double t) { return value.timestamp < t; });
      if (first == imu_samples_.begin() || end == imu_samples_.end()) break;
      --first;
      data.imu_samples.assign(first, std::next(end));
    }
    interpolateJoints(joint_samples_, data.timestamp, joint_max_gap_, data.joint_positions);
    data.support = contact_tracker_->at(data.timestamp);
    if (config_.use_terrain_factor) {
      Eigen::Vector3d left, right;
      if (optimizer_->footPositions(data.joint_positions, data.vio_pose, left, right)) {
        data.left_terrain = queryElevationMap(left, data.timestamp);
        data.right_terrain = queryElevationMap(right, data.timestamp);
      }
    }
    try {
      if (!optimizer_->processSensorData(data)) {
        diagnostic_reason_ = "IMU interval missing/invalid; no state published";
        // Do not retry this keyframe with a contact tracker already committed to its time.
        vio_samples_.pop_front();
        optimizer_->reset();
        processed_stamp_ = -1;
        continue;
      }
    } catch (const std::exception& error) {
      optimizer_->reset();
      processed_stamp_ = -1;
      vio_samples_.pop_front();
      diagnostic_reason_ = std::string("Optimization failed; reset: ") + error.what();
      continue;
    }
    vio_samples_.pop_front();
    processed_stamp_ = data.timestamp;
    while (imu_samples_.size() > 2 && imu_samples_[1].timestamp <= processed_stamp_) imu_samples_.pop_front();
    while (joint_samples_.size() > 2 && joint_samples_[1].timestamp <= processed_stamp_) joint_samples_.pop_front();
    EstimatorOutput output;
    if (optimizer_->getLatestOutput(output)) {
      publishFusedOdometry(output);
      publishContactState(output.contact_state);
      std_msgs::msg::UInt8 stable;
      stable.data = static_cast<uint8_t>(output.stable_support);
      stable_support_pub_->publish(stable);
      publishTF(output);
      diagnostic_reason_ = config_.contact_geometry_calibrated ? "Running" :
          "VIO/IMU only: contact geometry is not calibrated";
    }
  }
}

TerrainObservation CTExoStateNode::queryElevationMap(const Eigen::Vector3d& point, double stamp) {
  TerrainObservation result;
  std::lock_guard<std::mutex> lock(map_mutex_);
  if (!latest_grid_map_ || latest_grid_map_->header.frame_id != world_frame_) return result;
  const double age = stamp - seconds(latest_grid_map_->header.stamp);
  if (age < 0 || age > map_timeout_) return result;
  grid_map::GridMap map;
  if (!grid_map::GridMapRosConverter::fromMessage(*latest_grid_map_, map)) return result;
  grid_map::Position pos(point.x(), point.y());
  const double h = 2.0 * map.getResolution();
  if (!map.isInside(pos) || !map.isInside(pos + grid_map::Position(h, 0)) ||
      !map.isInside(pos - grid_map::Position(h, 0)) ||
      !map.isInside(pos + grid_map::Position(0, h)) ||
      !map.isInside(pos - grid_map::Position(0, h))) return result;
  try {
    const double z = map.atPosition("elevation", pos);
    const double dx = (map.atPosition("elevation", pos + grid_map::Position(h, 0)) -
                       map.atPosition("elevation", pos - grid_map::Position(h, 0))) / (2 * h);
    const double dy = (map.atPosition("elevation", pos + grid_map::Position(0, h)) -
                       map.atPosition("elevation", pos - grid_map::Position(0, h))) / (2 * h);
    if (!std::isfinite(z) || !std::isfinite(dx) || !std::isfinite(dy)) return result;
    result.point = {point.x(), point.y(), z};
    result.normal = Eigen::Vector3d(-dx, -dy, 1).normalized();
    if (!map.exists("variance")) return result;
    const double variance = map.atPosition("variance", pos);
    if (!std::isfinite(variance) || variance <= 0) return result;
    result.sigma = std::sqrt(variance);
    result.valid = true;
  } catch (const std::exception&) {}
  return result;
}

void CTExoStateNode::publishFusedOdometry(const EstimatorOutput& output) {
  nav_msgs::msg::Odometry msg;
  msg.header.stamp = messageTime(output.timestamp);
  msg.header.frame_id = world_frame_;
  msg.child_frame_id = pelvis_frame_;
  const auto position = output.pose.translation();
  msg.pose.pose.position.x = position.x();
  msg.pose.pose.position.y = position.y();
  msg.pose.pose.position.z = position.z();
  const auto q = output.pose.rotation().toQuaternion();
  msg.pose.pose.orientation.w = q.w();
  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();
  // Pose3 right-local [rotation,translation] -> ROS world [position,rotation].
  const Eigen::Matrix3d rotation = output.pose.rotation().matrix();
  Eigen::Matrix<double, 6, 6> transform = Eigen::Matrix<double, 6, 6>::Zero();
  transform.block<3, 3>(0, 3) = rotation;
  transform.block<3, 3>(3, 0) = rotation;
  const Eigen::Matrix<double, 6, 6> covariance = transform * output.pose_cov * transform.transpose();
  for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j) {
    msg.pose.covariance[6 * i + j] = covariance(i, j);
  }
  const Eigen::Vector3d velocity = rotation.transpose() * output.velocity;
  msg.twist.twist.linear.x = velocity.x();
  msg.twist.twist.linear.y = velocity.y();
  msg.twist.twist.linear.z = velocity.z();
  const Eigen::Matrix3d velocity_cov = rotation.transpose() * output.velocity_cov * rotation;
  for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) {
    msg.twist.covariance[6 * i + j] = velocity_cov(i, j);
  }
  // Angular velocity is not an estimated output of this graph.
  for (int i = 3; i < 6; ++i) msg.twist.covariance[6 * i + i] = 1e6;
  fused_odom_pub_->publish(msg);
}

void CTExoStateNode::publishContactState(ContactState state) {
  std_msgs::msg::UInt8 msg;
  msg.data = static_cast<uint8_t>(state);
  contact_pub_->publish(msg);
}

void CTExoStateNode::publishDiagnostics() {
  std::lock_guard<std::mutex> lock(data_mutex_);
  diagnostic_msgs::msg::DiagnosticArray msg;
  msg.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "exo_state_estimator";
  EstimatorOutput output;
  const bool has_output = optimizer_->getLatestOutput(output);
  const double age = has_output ? now().seconds() - output.timestamp : -1.0;
  const bool fresh = has_output && age >= 0 && age <= input_timeout_;
  status.level = fresh && config_.contact_geometry_calibrated ? status.OK : status.WARN;
  status.message = fresh ? diagnostic_reason_ : "No fresh estimate: " + diagnostic_reason_;
  auto add = [&](const std::string& key, const std::string& value) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key; kv.value = value; status.values.push_back(kv);
  };
  add("state_age_s", std::to_string(age));
  add("gait_contact_valid", has_output && output.contact_state != ContactState::UNKNOWN ? "true" : "false");
  add("optimize_time_ms", std::to_string(output.optimize_time_ms));
  add("num_factors", std::to_string(output.num_factors));
  add("num_states", std::to_string(output.num_states));
  add("left_landmark", std::to_string(output.left_landmark));
  add("right_landmark", std::to_string(output.right_landmark));
  msg.status.push_back(status);
  diag_pub_->publish(msg);
  if (!fresh) {
    publishContactState(ContactState::UNKNOWN);
    std_msgs::msg::UInt8 stable;
    stable.data = static_cast<uint8_t>(ContactState::UNKNOWN);
    stable_support_pub_->publish(stable);
  }
}

void CTExoStateNode::publishTF(const EstimatorOutput& output) {
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = messageTime(output.timestamp);
  tf.header.frame_id = world_frame_;
  tf.child_frame_id = pelvis_frame_;
  tf.transform.translation.x = output.pose.x();
  tf.transform.translation.y = output.pose.y();
  tf.transform.translation.z = output.pose.z();
  const auto q = output.pose.rotation().toQuaternion();
  tf.transform.rotation.w = q.w();
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf_broadcaster_->sendTransform(tf);
}

}  // namespace exo_state_estimator
