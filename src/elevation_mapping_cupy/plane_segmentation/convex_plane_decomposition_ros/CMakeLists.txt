cmake_minimum_required(VERSION 3.10)
project(convex_plane_decomposition_ros)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(convex_plane_decomposition REQUIRED)
find_package(convex_plane_decomposition_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

set(dependencies
  convex_plane_decomposition
  convex_plane_decomposition_msgs
  diagnostic_msgs
  geometry_msgs
  grid_map_core
  grid_map_msgs
  grid_map_ros
  rclcpp
  std_msgs
  tf2
  tf2_eigen
  tf2_ros
  visualization_msgs
)

###########
## Build ##
###########

add_library(${PROJECT_NAME}
  src/ConvexPlaneDecompositionRos.cpp
  src/MessageConversion.cpp
  src/ParameterLoading.cpp
  src/RosVisualizations.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(plane_segmentation_node src/ConvexPlaneDecompositionNode.cpp)
target_compile_features(plane_segmentation_node PUBLIC cxx_std_17)
target_link_libraries(plane_segmentation_node ${PROJECT_NAME})
ament_target_dependencies(plane_segmentation_node rclcpp)

#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include/${PROJECT_NAME}
)
install(TARGETS plane_segmentation_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config data launch rviz DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)
install(FILES PERFORMANCE.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/synthetic_grid_map_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME synthetic_grid_map_publisher
)

ament_export_dependencies(${dependencies})
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_plane_segmentation_node test/TestPlaneSegmentationNode.cpp)
  target_link_libraries(test_plane_segmentation_node ${PROJECT_NAME})
  ament_target_dependencies(test_plane_segmentation_node
    diagnostic_msgs
    grid_map_core
    grid_map_msgs
    grid_map_ros
    rclcpp
    tf2_ros
  )
endif()

ament_package()
