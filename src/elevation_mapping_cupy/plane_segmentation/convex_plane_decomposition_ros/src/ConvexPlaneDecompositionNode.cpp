#include "convex_plane_decomposition_ros/ConvexPlaneDecompositionRos.h"

#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<convex_plane_decomposition::ConvexPlaneExtractionROS>());
  } catch (const std::exception& exception) {
    RCLCPP_FATAL(rclcpp::get_logger("plane_segmentation_node"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
