#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "exo_state_estimator/msg/gait_event.hpp"
#include "exo_state_estimator/msg/gait_contact_state.hpp"
#include "exo_state_estimator/contact_detector.hpp"
#include "exo_state_estimator/factor_graph_optimizer.hpp"
#include "exo_state_estimator/measurement_buffer.hpp"

namespace exo_state_estimator {

class CTExoStateNode : public rclcpp::Node {
 public:
  explicit CTExoStateNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  void loadParameters();
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void vioOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void gridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg);
  void gaitEventCallback(const msg::GaitEvent::SharedPtr msg);
  void gaitContactCallback(const msg::GaitContactState::SharedPtr msg);
  void optimizeTimerCallback();
  void resetInputs();
  void publishFusedOdometry(const EstimatorOutput& output);
  void publishContactState(ContactState state);
  void publishDiagnostics();
  void publishTF(const EstimatorOutput& output);
  TerrainObservation queryElevationMap(const Eigen::Vector3d& point, double stamp);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vio_odom_sub_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_sub_;
  rclcpp::Subscription<msg::GaitEvent>::SharedPtr gait_event_sub_;
  rclcpp::Subscription<msg::GaitContactState>::SharedPtr gait_contact_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_odom_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr contact_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr stable_support_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr optimize_timer_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
  std::unique_ptr<FactorGraphOptimizer> optimizer_;
  std::unique_ptr<GaitContactTracker> contact_tracker_;
  EstimatorConfig config_;
  std::mutex data_mutex_;
  std::deque<ImuSample> imu_samples_;
  std::deque<JointSample> joint_samples_;
  std::deque<SensorData> vio_samples_;
  std::mutex map_mutex_;
  grid_map_msgs::msg::GridMap::SharedPtr latest_grid_map_;
  std::vector<std::string> active_joint_names_{"left_hip3", "left_knee", "left_ankle",
                                              "right_hip3", "right_knee", "right_ankle"};
  std::string world_frame_;
  std::string pelvis_frame_;
  std::string vio_body_frame_;
  double keyframe_period_ = .04;
  double last_keyframe_stamp_ = -1;
  double fusion_delay_ = .1;
  double input_timeout_ = .5;
  double joint_max_gap_ = .03;
  double map_timeout_ = .3;
  double last_ros_time_ = -1;
  double last_vio_stamp_ = -1;
  double processed_stamp_ = -1;
  std::string diagnostic_reason_ = "Waiting for pelvis VIO";
};

}  // namespace exo_state_estimator
