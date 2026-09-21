#include "exo_state_estimator/ct_exo_state_node.hpp"

#include <chrono>
#include <fstream>
#include <sstream>

#include <tf2/LinearMath/Quaternion.h>

// grid_map 只在 .cpp 中使用, 避免 Eigen 插件冲突
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

namespace exo_state_estimator {

CTExoStateNode::CTExoStateNode() : Node("ct_exo_state_node") {
  RCLCPP_INFO(this->get_logger(), "Initializing CT-ExoState estimator...");

  loadParameters();

  config_ = buildConfig();

  // 初始化优化器
  optimizer_ = std::make_unique<FactorGraphOptimizer>();
  if (!optimizer_->init(config_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize optimizer");
    rclcpp::shutdown();
    return;
  }

  // 初始化接触检测器
  ThresholdContactDetector::Config det_config;
  contact_detector_ = std::make_unique<ThresholdContactDetector>(det_config);

  // 订阅
  // 独立 IMU 传感器 (非 VIO 内置的 D435i IMU)
  // 通过 launch 文件 remappings 映射到实际 topic
  std::string imu_topic = this->declare_parameter("imu_topic", "/imu/standalone");
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      std::bind(&CTExoStateNode::imuCallback, this, std::placeholders::_1));

  // 状态估计必须使用编码器实测值，而不是 CPG 期望轨迹。话题做成参数，
  // 便于 rosbag 回放时重映射，但默认接 ros2_control 的 joint_state_broadcaster。
  std::string joint_state_topic =
      this->declare_parameter("joint_state_topic", "/joint_states");
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic, rclcpp::SensorDataQoS(),
      std::bind(&CTExoStateNode::jointStateCallback, this,
                std::placeholders::_1));

  vio_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/ov_msckf/odomimu", rclcpp::SensorDataQoS(),
      std::bind(&CTExoStateNode::vioOdometryCallback, this,
                std::placeholders::_1));

  grid_map_sub_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
      "/elevation_mapping_node/elevation_map_raw", rclcpp::QoS(1).reliable(),
      std::bind(&CTExoStateNode::gridMapCallback, this,
                std::placeholders::_1));

  cpg_state_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/cpg/state", rclcpp::QoS(10),
      std::bind(&CTExoStateNode::cpgStateCallback, this,
                std::placeholders::_1));

  // 发布
  fused_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/fused/odometry", rclcpp::SensorDataQoS());

  contact_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
      "/fused/contact_state", rclcpp::QoS(10));

  diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/fused/diagnostics", rclcpp::QoS(1));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // 定时优化触发 (200Hz)
  double optimize_freq = this->declare_parameter("optimize_frequency", 200.0);
  optimize_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / optimize_freq),
      std::bind(&CTExoStateNode::optimizeTimerCallback, this));

  RCLCPP_INFO(this->get_logger(), "CT-ExoState estimator started at %.0f Hz",
              optimize_freq);
}

void CTExoStateNode::loadParameters() {
  world_frame_ = this->declare_parameter("world_frame", "global");
  pelvis_frame_ = this->declare_parameter("pelvis_frame", "pelvis_fused");
  imu_frame_ = this->declare_parameter("imu_frame", "imu");

  config_.urdf_path =
      this->declare_parameter("urdf_path", std::string(""));
  config_.lag_seconds = this->declare_parameter("lag_seconds", 5.0);
  config_.frame_dt = this->declare_parameter("frame_dt", 0.005);

  config_.terrain_height = this->declare_parameter("terrain_height", 0.0);
  config_.terrain_sigma_sensor =
      this->declare_parameter("terrain_sigma_sensor", 0.01);
  config_.kinematic_w_valid =
      this->declare_parameter("kinematic_w_valid", 0.999);

  // IMU 预留参数
  config_.use_imu_factor =
      this->declare_parameter("use_imu_factor", false);

  config_.imu_accel_noise =
      this->declare_parameter("imu_accel_noise", 0.01);
  config_.imu_gyro_noise =
      this->declare_parameter("imu_gyro_noise", 0.001);
  config_.imu_accel_bias_rw =
      this->declare_parameter("imu_accel_bias_rw", 0.0002);
  config_.imu_gyro_bias_rw =
      this->declare_parameter("imu_gyro_bias_rw", 0.00003);

  // VIO 协方差缩放
  double vio_pos_sigma = this->declare_parameter("vio_pos_sigma", 0.05);
  double vio_rot_sigma = this->declare_parameter("vio_rot_sigma", 0.1);
  config_.vio_sigmas << vio_rot_sigma, vio_rot_sigma, vio_rot_sigma,
      vio_pos_sigma, vio_pos_sigma, vio_pos_sigma;

  // 运动学协方差
  double kin_stick_sigma = this->declare_parameter("kinematic_stick_sigma", 0.05);
  config_.kinematic_cov_stick =
      Eigen::Vector3d::Constant(kin_stick_sigma * kin_stick_sigma)
          .asDiagonal();

  // 先验噪声
  double prior_pos = this->declare_parameter("prior_pos_sigma", 0.01);
  double prior_rot = this->declare_parameter("prior_rot_sigma", 0.01);
  config_.prior_sigmas << prior_rot, prior_rot, prior_rot, prior_pos,
      prior_pos, prior_pos;
}

EstimatorConfig CTExoStateNode::buildConfig() const { return config_; }

void CTExoStateNode::imuCallback(
    const sensor_msgs::msg::Imu::SharedPtr msg) {
  // 独立 IMU 传感器 (非 VIO 内置的 D435i IMU)
  // 当 use_imu_factor=true 时, 数据会被送入 IMU 预积分器
  std::lock_guard<std::mutex> lock(data_mutex_);
  current_data_.accel =
      Eigen::Vector3d(msg->linear_acceleration.x,
                      msg->linear_acceleration.y,
                      msg->linear_acceleration.z);
  current_data_.gyro = Eigen::Vector3d(msg->angular_velocity.x,
                                       msg->angular_velocity.y,
                                       msg->angular_velocity.z);
  current_data_.imu_valid = true;
  current_data_.timestamp =
      msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
}

void CTExoStateNode::jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);

  // 按 active_joint_names_ 顺序提取关节值
  current_data_.joint_positions.resize(active_joint_names_.size());
  current_data_.joint_velocities.resize(active_joint_names_.size());

  for (size_t i = 0; i < active_joint_names_.size(); ++i) {
    auto it = std::find(msg->name.begin(), msg->name.end(),
                        active_joint_names_[i]);
    if (it != msg->name.end()) {
      size_t idx = std::distance(msg->name.begin(), it);
      current_data_.joint_positions[i] = msg->position[idx];
      if (idx < msg->velocity.size()) {
        current_data_.joint_velocities[i] = msg->velocity[idx];
      }
    }
  }
}

void CTExoStateNode::vioOdometryCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);

  const auto& p = msg->pose.pose.position;
  const auto& q = msg->pose.pose.orientation;
  gtsam::Rot3 rot(gtsam::Quaternion(q.w, q.x, q.y, q.z));
  gtsam::Point3 pos(p.x, p.y, p.z);
  current_data_.vio_pose = gtsam::Pose3(rot, pos);

  // 提取协方差 (6x6 from 6x6 row-major)
  Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Zero();
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j)
      cov(i, j) = msg->pose.covariance[i * 6 + j];
  current_data_.vio_covariance = cov;
  current_data_.vio_valid = true;
}

void CTExoStateNode::gridMapCallback(
    const grid_map_msgs::msg::GridMap::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(map_mutex_);
  latest_grid_map_ = msg;
}

void CTExoStateNode::cpgStateCallback(
    const std_msgs::msg::String::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  current_data_.cpg_state = msg->data;

  // 基于 CPG 状态粗略推断接触 (辅助)
  if (msg->data == "IDLE") {
    current_data_.contact_state = ContactState::DOUBLE_STANCE;
  }
}

void CTExoStateNode::optimizeTimerCallback() {
  SensorData data;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    data = current_data_;
  }

  // 查询高程图
  if (optimizer_->getLatestOutput(latest_output_) || frame_count_ == 0) {
    // 用最新估计的位姿查询高程图
    double x = data.vio_valid ? data.vio_pose.x() : 0;
    double y = data.vio_valid ? data.vio_pose.y() : 0;
    queryElevationMap(x, y, data.terrain_z, data.terrain_normal,
                      data.terrain_sigma, data.terrain_valid);
  }

  // 接触检测
  ContactDetectionInput det_input;
  det_input.cpg_state = data.cpg_state;
  if (data.joint_positions.size() >= 6) {
    // 用关节速度的垂直分量作为辅助
    det_input.foot_vel_left_z = data.joint_velocities[2];   // ankle
    det_input.foot_vel_right_z = data.joint_velocities[5];  // ankle
  }
  ContactDetectionOutput det_output = contact_detector_->detect(det_input);
  data.contact_state = det_output.state;

  // 送入优化器
  optimizer_->processSensorData(data);

  // 发布结果
  EstimatorOutput output;
  if (optimizer_->getLatestOutput(output)) {
    publishFusedOdometry(output);
    publishContactState(output.contact_state);
    publishDiagnostics(output);
    publishTF(output);
  }
}

void CTExoStateNode::queryElevationMap(double x, double y, double& z_out,
                                       Eigen::Vector3d& normal_out,
                                       double& sigma_out, bool& valid_out) {
  // 默认输出
  z_out = config_.terrain_height;
  normal_out = config_.terrain_normal;
  sigma_out = config_.terrain_sigma_sensor;
  valid_out = false;

  std::lock_guard<std::mutex> lock(map_mutex_);
  if (!latest_grid_map_) return;

  // 按需转换: 原始消息 -> grid_map::GridMap 对象
  grid_map::GridMap map;
  if (!grid_map::GridMapRosConverter::fromMessage(*latest_grid_map_, map)) return;

  grid_map::Position query_pos(x, y);

  // 检查查询点是否在地图范围内
  if (!map.isInside(query_pos)) return;

  // 查询 elevation 层 (最近邻, grid_map_core 2.0.1 插值方法为 private)
  float elevation = 0.0f;
  try {
    elevation = map.atPosition("elevation", query_pos);
  } catch (...) {
    return;
  }
  if (std::isnan(elevation)) return;

  z_out = static_cast<double>(elevation);

  // 从 elevation 梯度计算法向量 (中心差分, 步长=2格)
  double resolution = map.getResolution();
  double h = resolution * 2.0;

  grid_map::Position pos_xp(x + h, y), pos_xm(x - h, y);
  grid_map::Position pos_yp(x, y + h), pos_ym(x, y - h);

  if (!map.isInside(pos_xp) || !map.isInside(pos_xm) ||
      !map.isInside(pos_yp) || !map.isInside(pos_ym)) {
    // 边界处用默认法向量, 但 elevation 有效
    valid_out = true;
    return;
  }

  float z_xp, z_xm, z_yp, z_ym;
  try {
    z_xp = map.atPosition("elevation", pos_xp);
    z_xm = map.atPosition("elevation", pos_xm);
    z_yp = map.atPosition("elevation", pos_yp);
    z_ym = map.atPosition("elevation", pos_ym);
  } catch (...) {
    valid_out = true;
    return;
  }

  if (std::isnan(z_xp) || std::isnan(z_xm) || std::isnan(z_yp) || std::isnan(z_ym)) {
    valid_out = true;
    return;
  }

  // 梯度 -> 法向量: n = normalize([-dz/dx, -dz/dy, 1])
  double dzdx = static_cast<double>(z_xp - z_xm) / (2.0 * h);
  double dzdy = static_cast<double>(z_yp - z_ym) / (2.0 * h);
  normal_out = Eigen::Vector3d(-dzdx, -dzdy, 1.0).normalized();

  // 查询 variance 层获取不确定度
  if (map.exists("variance")) {
    try {
      float variance = map.atPosition("variance", query_pos);
      if (!std::isnan(variance) && variance > 0) {
        sigma_out = std::sqrt(static_cast<double>(variance));
      }
    } catch (...) {}
  }

  valid_out = true;
}

void CTExoStateNode::publishFusedOdometry(const EstimatorOutput& output) {
  auto msg = std::make_unique<nav_msgs::msg::Odometry>();
  msg->header.stamp = this->now();
  msg->header.frame_id = world_frame_;
  msg->child_frame_id = pelvis_frame_;

  msg->pose.pose.position.x = output.pose.x();
  msg->pose.pose.position.y = output.pose.y();
  msg->pose.pose.position.z = output.pose.z();

  gtsam::Quaternion q = output.pose.rotation().toQuaternion();
  msg->pose.pose.orientation.w = q.w();
  msg->pose.pose.orientation.x = q.x();
  msg->pose.pose.orientation.y = q.y();
  msg->pose.pose.orientation.z = q.z();

  // 协方差
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j)
      msg->pose.covariance[i * 6 + j] = output.pose_cov(i, j);

  // 速度 (body frame, 来自 IMU 预积分优化)
  msg->twist.twist.linear.x = output.velocity.x();
  msg->twist.twist.linear.y = output.velocity.y();
  msg->twist.twist.linear.z = output.velocity.z();

  fused_odom_pub_->publish(std::move(msg));
}

void CTExoStateNode::publishContactState(ContactState state) {
  auto msg = std::make_unique<std_msgs::msg::UInt8>();
  msg->data = static_cast<uint8_t>(state);
  contact_pub_->publish(std::move(msg));
}

void CTExoStateNode::publishDiagnostics(const EstimatorOutput& output) {
  auto msg = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();
  msg->header.stamp = this->now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "exo_state_estimator";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = "Running";

  diagnostic_msgs::msg::KeyValue kv1;
  kv1.key = "optimize_time_ms";
  kv1.value = std::to_string(output.optimize_time_ms);
  status.values.push_back(kv1);

  diagnostic_msgs::msg::KeyValue kv2;
  kv2.key = "num_factors";
  kv2.value = std::to_string(output.num_factors);
  status.values.push_back(kv2);

  diagnostic_msgs::msg::KeyValue kv3;
  kv3.key = "slip_left";
  kv3.value = output.slip_detected_left ? "true" : "false";
  status.values.push_back(kv3);

  diagnostic_msgs::msg::KeyValue kv4;
  kv4.key = "slip_right";
  kv4.value = output.slip_detected_right ? "true" : "false";
  status.values.push_back(kv4);

  msg->status.push_back(status);
  diag_pub_->publish(std::move(msg));
}

void CTExoStateNode::publishTF(const EstimatorOutput& output) {
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = this->now();
  tf.header.frame_id = world_frame_;
  tf.child_frame_id = pelvis_frame_;

  tf.transform.translation.x = output.pose.x();
  tf.transform.translation.y = output.pose.y();
  tf.transform.translation.z = output.pose.z();

  gtsam::Quaternion q = output.pose.rotation().toQuaternion();
  tf.transform.rotation.w = q.w();
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();

  tf_broadcaster_->sendTransform(tf);
}

}  // namespace exo_state_estimator
