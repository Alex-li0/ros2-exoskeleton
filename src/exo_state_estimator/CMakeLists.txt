cmake_minimum_required(VERSION 3.8)
project(exo_state_estimator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================
# 依赖查找
# ============================================================
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GaitEvent.msg"
  "msg/GaitContactState.msg"
  DEPENDENCIES std_msgs
)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(GTSAM REQUIRED)
find_package(GTSAM_UNSTABLE REQUIRED)
find_package(orocos_kdl REQUIRED)

# ============================================================
# 头文件路径
# ============================================================
include_directories(
  include
  ${GTSAM_INCLUDE_DIR}
  ${orocos_kdl_INCLUDE_DIRS}
)

# ============================================================
# 库 (不含 grid_map, 避免 Eigen 插件冲突)
# ============================================================
add_library(exo_state_lib SHARED
  src/fk_solver.cpp
  src/contact_detector.cpp
  src/factor_graph_optimizer.cpp
)
target_link_libraries(exo_state_lib
  gtsam
  gtsam_unstable
  ${orocos_kdl_LIBRARIES}
)
ament_target_dependencies(exo_state_lib
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  urdf
  kdl_parser
)

# ============================================================
# 可执行文件 (使用 grid_map_core, 手动设置 Eigen 插件)
# ============================================================
add_executable(ct_exo_state_node
  src/main.cpp
  src/ct_exo_state_node.cpp
)
target_link_libraries(ct_exo_state_node exo_state_lib)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(ct_exo_state_node "${cpp_typesupport_target}")

# grid_map_core 的 Eigen 插件需要在编译时定义
# 不用 find_package(grid_map_core) 避免全局 add_definitions
find_package(grid_map_core QUIET)
# 移除 grid_map_core 设置的全局定义 (仅影响 exo_state_lib)
get_directory_property(_dir_defs COMPILE_DEFINITIONS)
list(REMOVE_ITEM _dir_defs
  "EIGEN_FUNCTORS_PLUGIN=\"grid_map_core/eigen_plugins/FunctorsPlugin.hpp\""
  "EIGEN_DENSEBASE_PLUGIN=\"grid_map_core/eigen_plugins/DenseBasePlugin.hpp\""
)
set_directory_properties(PROPERTIES COMPILE_DEFINITIONS "${_dir_defs}")

# 仅对 node 目标添加 grid_map 编译定义
target_compile_definitions(ct_exo_state_node PRIVATE
  EIGEN_FUNCTORS_PLUGIN="grid_map_core/eigen_plugins/FunctorsPlugin.hpp"
  EIGEN_DENSEBASE_PLUGIN="grid_map_core/eigen_plugins/DenseBasePlugin.hpp"
)

ament_target_dependencies(ct_exo_state_node
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  diagnostic_msgs
  tf2_ros
  tf2_geometry_msgs
  grid_map_msgs
  grid_map_ros
  grid_map_core
)

# ============================================================
# 安装
# ============================================================
install(TARGETS ct_exo_state_node exo_state_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# ============================================================
# 测试
# ============================================================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_estimator test/test_estimator.cpp)
  target_link_libraries(test_estimator exo_state_lib)
  ament_target_dependencies(test_estimator nav_msgs urdf kdl_parser)
  target_compile_definitions(test_estimator PRIVATE
    TEST_URDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../robot_display/urdf/robot_display.urdf")
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_test(test_node_io
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_node_io.py"
      --executable "$<TARGET_FILE:ct_exo_state_node>"
    TIMEOUT 40
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
