#include <rclcpp/rclcpp.hpp>
#include "exo_state_estimator/ct_exo_state_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<exo_state_estimator::CTExoStateNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
