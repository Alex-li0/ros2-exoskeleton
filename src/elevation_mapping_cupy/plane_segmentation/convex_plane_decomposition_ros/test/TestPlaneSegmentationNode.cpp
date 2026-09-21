#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <gtest/gtest.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "convex_plane_decomposition_ros/ConvexPlaneDecompositionRos.h"

namespace convex_plane_decomposition {
namespace {

using namespace std::chrono_literals;

class PlaneSegmentationNodeTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

grid_map_msgs::msg::GridMap makeFlatMap(const rclcpp::Time& stamp, bool includeElevation) {
  grid_map::GridMap map(includeElevation ? std::vector<std::string>{"elevation"} : std::vector<std::string>{"wrong_layer"});
  map.setFrameId("map");
  map.setTimestamp(stamp.nanoseconds());
  map.setGeometry(grid_map::Length(1.0, 1.0), 0.05, grid_map::Position(0.0, 0.0));
  map.get(includeElevation ? "elevation" : "wrong_layer").setConstant(0.2F);
  if (includeElevation) {
    map.get("elevation").block<4, 4>(4, 4).setConstant(std::numeric_limits<float>::quiet_NaN());
  }
  return *grid_map::GridMapRosConverter::toMessage(map);
}

TEST_F(PlaneSegmentationNodeTest, PreservesHeaderGeometryAndRejectsMissingLayer) {
  const std::string inputTopic = "/plane_segmentation_test/input";
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("input_topic", inputTopic),
      rclcpp::Parameter("target_frame", "map"),
      rclcpp::Parameter("roi.center_frame", "map"),
      rclcpp::Parameter("processing_rate_hz", 30.0),
      rclcpp::Parameter("input_timeout_sec", 5.0),
      rclcpp::Parameter("roi.side_length", 0.8),
      rclcpp::Parameter("minimum_valid_cells", 10),
      rclcpp::Parameter("minimum_region_area", 0.001),
      // Deliberately differ from the 0.05 m input to cover validity-mask resampling.
      rclcpp::Parameter("preprocessing.resolution", 0.1),
      rclcpp::Parameter("sliding_window_plane_extractor.include_ransac_refinement", false),
  });

  auto segmentationNode = std::make_shared<ConvexPlaneExtractionROS>(options);
  auto testNode = std::make_shared<rclcpp::Node>("plane_segmentation_test_driver");
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  auto publisher = testNode->create_publisher<grid_map_msgs::msg::GridMap>(inputTopic, qos);

  convex_plane_decomposition_msgs::msg::PlanarTerrain::SharedPtr terrain;
  grid_map_msgs::msg::GridMap::SharedPtr segmentationMap;
  diagnostic_msgs::msg::DiagnosticArray::SharedPtr diagnostics;
  auto terrainSubscription = testNode->create_subscription<convex_plane_decomposition_msgs::msg::PlanarTerrain>(
      "/plane_segmentation_node/planar_terrain", qos,
      [&terrain](convex_plane_decomposition_msgs::msg::PlanarTerrain::SharedPtr message) { terrain = std::move(message); });
  auto diagnosticSubscription = testNode->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/plane_segmentation_node/diagnostics", qos,
      [&diagnostics](diagnostic_msgs::msg::DiagnosticArray::SharedPtr message) { diagnostics = std::move(message); });
  auto segmentationMapSubscription = testNode->create_subscription<grid_map_msgs::msg::GridMap>(
      "/plane_segmentation_node/segmentation_map", qos,
      [&segmentationMap](grid_map_msgs::msg::GridMap::SharedPtr message) { segmentationMap = std::move(message); });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(segmentationNode);
  executor.add_node(testNode);
  for (int i = 0; i < 10; ++i) {
    executor.spin_some();
    std::this_thread::sleep_for(20ms);
  }

  const auto inputStamp = testNode->now();
  publisher->publish(makeFlatMap(inputStamp, true));
  for (int i = 0; i < 100 && (terrain == nullptr || segmentationMap == nullptr); ++i) {
    executor.spin_some();
    std::this_thread::sleep_for(20ms);
  }

  ASSERT_NE(terrain, nullptr);
  EXPECT_EQ(terrain->gridmap.header.frame_id, "map");
  EXPECT_EQ(terrain->gridmap.header.stamp.sec, inputStamp.nanoseconds() / 1000000000LL);
  EXPECT_EQ(terrain->gridmap.header.stamp.nanosec,
            static_cast<uint32_t>(inputStamp.nanoseconds() % 1000000000LL));
  ASSERT_EQ(terrain->planar_regions.size(), 1U);
  const auto& region = terrain->planar_regions.front();
  EXPECT_NEAR(std::abs(region.plane_parameters.orientation.w), 1.0, 1e-6);
  ASSERT_FALSE(region.boundary.outer_boundary.points.empty());
  for (const auto& point : region.boundary.outer_boundary.points) {
    EXPECT_TRUE(std::isfinite(point.x));
    EXPECT_TRUE(std::isfinite(point.y));
  }

  ASSERT_NE(segmentationMap, nullptr);
  grid_map::GridMap debugMap;
  ASSERT_TRUE(grid_map::GridMapRosConverter::fromMessage(*segmentationMap, debugMap));
  ASSERT_TRUE(debugMap.exists("elevation_raw"));
  ASSERT_TRUE(debugMap.exists("plane_visualization_height"));
  bool foundUnobservedCell = false;
  for (Eigen::Index row = 0; row < debugMap.get("elevation_raw").rows(); ++row) {
    for (Eigen::Index col = 0; col < debugMap.get("elevation_raw").cols(); ++col) {
      if (!std::isfinite(debugMap.get("elevation_raw")(row, col))) {
        foundUnobservedCell = true;
        EXPECT_FALSE(std::isfinite(debugMap.get("plane_visualization_height")(row, col)));
      }
    }
  }
  EXPECT_TRUE(foundUnobservedCell);

  terrain.reset();
  diagnostics.reset();
  publisher->publish(makeFlatMap(testNode->now(), false));
  for (int i = 0; i < 100 &&
                  (diagnostics == nullptr || diagnostics->status.empty() ||
                   diagnostics->status.front().message.find("missing") == std::string::npos);
       ++i) {
    executor.spin_some();
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_EQ(terrain, nullptr);
  ASSERT_NE(diagnostics, nullptr);
  ASSERT_FALSE(diagnostics->status.empty());
  EXPECT_NE(diagnostics->status.front().message.find("missing"), std::string::npos);

  diagnostics.reset();
  auto noTransformMap = makeFlatMap(testNode->now(), true);
  noTransformMap.header.frame_id = "frame_without_transform";
  publisher->publish(noTransformMap);
  for (int i = 0; i < 100 &&
                  (diagnostics == nullptr || diagnostics->status.empty() ||
                   diagnostics->status.front().message.find("TF unavailable") == std::string::npos);
       ++i) {
    executor.spin_some();
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_EQ(terrain, nullptr);
  ASSERT_NE(diagnostics, nullptr);
  ASSERT_FALSE(diagnostics->status.empty());
  EXPECT_NE(diagnostics->status.front().message.find("TF unavailable"), std::string::npos);

  (void)terrainSubscription;
  (void)diagnosticSubscription;
  (void)segmentationMapSubscription;
}

TEST_F(PlaneSegmentationNodeTest, CentersAxisAlignedSquareOnRuntimeConfigurableFrame) {
  const std::string inputTopic = "/plane_segmentation_test/tracked_input";
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("input_topic", inputTopic),
      rclcpp::Parameter("target_frame", "map"),
      rclcpp::Parameter("roi.center_frame", "imu_test"),
      rclcpp::Parameter("roi.side_length", 0.4),
      rclcpp::Parameter("processing_rate_hz", 30.0),
      rclcpp::Parameter("input_timeout_sec", 5.0),
      rclcpp::Parameter("minimum_valid_cells", 3),
      rclcpp::Parameter("minimum_region_area", 0.0),
      rclcpp::Parameter("preprocessing.resolution", 0.1),
      rclcpp::Parameter("sliding_window_plane_extractor.include_ransac_refinement", false),
  });

  auto segmentationNode = std::make_shared<ConvexPlaneExtractionROS>(options);
  auto testNode = std::make_shared<rclcpp::Node>("tracked_roi_test_driver");
  tf2_ros::StaticTransformBroadcaster broadcaster(testNode);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = testNode->now();
  transform.header.frame_id = "map";
  transform.child_frame_id = "imu_test";
  transform.transform.translation.x = 0.2;
  transform.transform.translation.y = -0.1;
  constexpr double yawHalfAngle = 0.7853981633974483;
  transform.transform.rotation.z = std::sin(yawHalfAngle);
  transform.transform.rotation.w = std::cos(yawHalfAngle);
  broadcaster.sendTransform(transform);

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  auto publisher = testNode->create_publisher<grid_map_msgs::msg::GridMap>(inputTopic, qos);
  grid_map_msgs::msg::GridMap::SharedPtr segmentationMap;
  auto subscription = testNode->create_subscription<grid_map_msgs::msg::GridMap>(
      "/plane_segmentation_node/segmentation_map", qos,
      [&segmentationMap](grid_map_msgs::msg::GridMap::SharedPtr message) { segmentationMap = std::move(message); });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(segmentationNode);
  executor.add_node(testNode);
  for (int i = 0; i < 20; ++i) {
    executor.spin_some();
    std::this_thread::sleep_for(20ms);
  }

  publisher->publish(makeFlatMap(testNode->now(), true));
  for (int i = 0; i < 100 && segmentationMap == nullptr; ++i) {
    executor.spin_some();
    std::this_thread::sleep_for(20ms);
  }
  ASSERT_NE(segmentationMap, nullptr);
  grid_map::GridMap debugMap;
  ASSERT_TRUE(grid_map::GridMapRosConverter::fromMessage(*segmentationMap, debugMap));
  EXPECT_NEAR(debugMap.getPosition().x(), 0.2, 0.051);
  EXPECT_NEAR(debugMap.getPosition().y(), -0.1, 0.051);
  EXPECT_NEAR(debugMap.getLength().x(), 0.4, 0.051);
  EXPECT_NEAR(debugMap.getLength().y(), 0.4, 0.051);

  const auto setResult = segmentationNode->set_parameter(rclcpp::Parameter("roi.side_length", 0.6));
  EXPECT_TRUE(setResult.successful);
  const auto setFrameResult = segmentationNode->set_parameter(rclcpp::Parameter("roi.center_frame", "map"));
  EXPECT_TRUE(setFrameResult.successful);
  const auto invalidResult = segmentationNode->set_parameter(rclcpp::Parameter("roi.side_length", 0.0));
  EXPECT_FALSE(invalidResult.successful);

  segmentationMap.reset();
  publisher->publish(makeFlatMap(testNode->now(), true));
  for (int i = 0; i < 100 && segmentationMap == nullptr; ++i) {
    executor.spin_some();
    std::this_thread::sleep_for(20ms);
  }
  ASSERT_NE(segmentationMap, nullptr);
  ASSERT_TRUE(grid_map::GridMapRosConverter::fromMessage(*segmentationMap, debugMap));
  EXPECT_NEAR(debugMap.getPosition().x(), 0.0, 0.051);
  EXPECT_NEAR(debugMap.getPosition().y(), 0.0, 0.051);
  EXPECT_NEAR(debugMap.getLength().x(), 0.6, 0.051);
  EXPECT_NEAR(debugMap.getLength().y(), 0.6, 0.051);

  (void)subscription;
}

}  // namespace
}  // namespace convex_plane_decomposition
