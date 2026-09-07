cmake_minimum_required(VERSION 3.8)
project(terrain_adaptive_cpg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TerrainModulation.msg"
  "msg/CPGParameterTarget.msg"
  "msg/CPGDebug.msg"
  "msg/TerrainSchedulerDebug.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

add_executable(terrain_parameter_scheduler src/terrain_parameter_scheduler.cpp)
target_include_directories(terrain_parameter_scheduler PUBLIC include)
ament_target_dependencies(terrain_parameter_scheduler nav_msgs rclcpp std_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(terrain_parameter_scheduler "${cpp_typesupport_target}")

install(TARGETS terrain_parameter_scheduler DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/terrain_synthetic_test_node.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_scheduler_core test/test_scheduler_core.cpp)
  target_include_directories(test_scheduler_core PUBLIC include)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
