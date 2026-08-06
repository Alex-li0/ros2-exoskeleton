#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "exo_state_estimator/contact_detector.hpp"
#include "exo_state_estimator/factor_graph_optimizer.hpp"

namespace exo_state_estimator {

class CTExoStateNode : public rclcpp::Node {
 public:
  CTExoStateNode();
  ~CTExoStateNode() override = default;

 private:
  // 参数加载
  void loadParameters();
  EstimatorConfig buildConfig() const;

  // 订阅回调
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void vioOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void gridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg);
  void cpgStateCallback(const std_msgs::msg::String::SharedPtr msg);

  // 定时优化触发
  void optimizeTimerCallback();

  // 发布
  void publishFusedOdometry(const EstimatorOutput& output);
  void publishContactState(ContactState state);
  void publishDiagnostics(const EstimatorOutput& output);
  void publishTF(const EstimatorOutput& output);

  // 查询高程图
  void queryElevationMap(double x, double y, double& z_out,
                         Eigen::Vector3d& normal_out, double& sigma_out,
                         bool& valid_out);

  // ROS2 订阅
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vio_odom_sub_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cpg_state_sub_;

  // ROS2 发布
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_odom_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr contact_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // 定时器
  rclcpp::TimerBase::SharedPtr optimize_timer_;

  // 核心组件
  std::unique_ptr<FactorGraphOptimizer> optimizer_;
  std::unique_ptr<ContactDetector> contact_detector_;
  EstimatorConfig config_;

  // 传感器数据缓冲
  SensorData current_data_;
  std::mutex data_mutex_;

  // 最新高程图数据缓存 (原始消息, 按需转换)
  std::mutex map_mutex_;
  grid_map_msgs::msg::GridMap::SharedPtr latest_grid_map_;

  // 关节名称映射
  std::vector<std::string> active_joint_names_{"left_hip3", "left_knee",
                                                "left_ankle", "right_hip3",
                                                "right_knee", "right_ankle"};

  // TF frame names
  std::string world_frame_ = "global";
  std::string pelvis_frame_ = "pelvis_fused";
  std::string imu_frame_ = "imu";

  // 内部状态
  EstimatorOutput latest_output_;
  int frame_count_ = 0;
};

}  // namespace exo_state_estimator
