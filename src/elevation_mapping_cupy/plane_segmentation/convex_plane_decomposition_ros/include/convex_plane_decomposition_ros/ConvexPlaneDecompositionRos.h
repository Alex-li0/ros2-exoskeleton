#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <convex_plane_decomposition/PlaneDecompositionPipeline.h>
#include <convex_plane_decomposition_msgs/msg/planar_terrain.hpp>

namespace convex_plane_decomposition {

class ConvexPlaneExtractionROS : public rclcpp::Node {
 public:
  explicit ConvexPlaneExtractionROS(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  using GridMapMessage = grid_map_msgs::msg::GridMap;

  void mapCallback(GridMapMessage::ConstSharedPtr message);
  void processLatestMap();
  void publishDiagnostics();
  rcl_interfaces::msg::SetParametersResult onSetParameters(const std::vector<rclcpp::Parameter>& parameters);
  bool validateMessage(const GridMapMessage& message, std::string& reason) const;
  bool prepareRoi(const GridMapMessage& message, grid_map::GridMap& roi, std::string& reason);
  void filterSmallRegions(PlanarTerrain& terrain) const;
  void recordProcessingTime(double milliseconds);

  std::string elevationMapTopic_;
  std::string elevationLayer_;
  std::string varianceLayer_;
  std::string targetFrameId_;
  std::string roiCenterFrameId_;
  double processingRateHz_{0.0};
  double inputTimeoutSec_{0.0};
  double tfTimeoutSec_{0.0};
  double roiSideLength_{0.0};
  int64_t minimumValidCells_{0};
  bool useVarianceLayer_{false};
  double maximumVariance_{0.0};
  double minimumRegionArea_{0.0};
  bool publishPlanarTerrain_{true};
  bool publishDebugOutputs_{true};
  double markerLineWidth_{0.005};

  std::unique_ptr<PlaneDecompositionPipeline> planeDecompositionPipeline_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;

  rclcpp::Subscription<GridMapMessage>::SharedPtr elevationMapSubscriber_;
  rclcpp::Publisher<GridMapMessage>::SharedPtr segmentationMapPublisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr boundaryPublisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr insetPublisher_;
  rclcpp::Publisher<convex_plane_decomposition_msgs::msg::PlanarTerrain>::SharedPtr regionPublisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnosticsPublisher_;
  rclcpp::TimerBase::SharedPtr processingTimer_;
  rclcpp::TimerBase::SharedPtr diagnosticsTimer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameterCallbackHandle_;

  mutable std::mutex latestMapMutex_;
  mutable std::mutex roiConfigMutex_;
  GridMapMessage::ConstSharedPtr latestMap_;
  bool latestMapPending_{false};
  rclcpp::Time latestReceiveTime_{0, 0, RCL_ROS_TIME};

  uint64_t receivedMaps_{0};
  uint64_t processedMaps_{0};
  uint64_t rejectedMaps_{0};
  uint64_t overwrittenMaps_{0};
  size_t lastRows_{0};
  size_t lastCols_{0};
  size_t lastOutputBytes_{0};
  std::string lastStatus_{"waiting for input"};
  std::vector<double> processingTimesMs_;
  std::chrono::steady_clock::time_point firstReceiveSteady_;
  std::chrono::steady_clock::time_point firstProcessSteady_;
};

}  // namespace convex_plane_decomposition
