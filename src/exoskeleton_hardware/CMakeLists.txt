cmake_minimum_required(VERSION 3.8)
project(exoskeleton_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/exo_hardware_interface.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  yaml_cpp_vendor
)

pluginlib_export_plugin_description_file(hardware_interface exoskeleton_hardware.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY urdf config launch
  DESTINATION share/${PROJECT_NAME}
)
# 可执行脚本 (ros2 run exoskeleton_hardware test_motor_control)
install(FILES scripts/test_motor_control.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME test_motor_control
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
              GROUP_READ GROUP_EXECUTE
              WORLD_READ WORLD_EXECUTE
)
# 辅助脚本 (低延迟配置等)
install(PROGRAMS scripts/set_low_latency.sh
  DESTINATION lib/${PROJECT_NAME}
)
# 控制监控脚本 (ros2 run exoskeleton_hardware plot_control_monitor)
install(FILES scripts/plot_control_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME plot_control_monitor
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
              GROUP_READ GROUP_EXECUTE
              WORLD_READ WORLD_EXECUTE
)
# 只读关节映射检查 (ros2 run exoskeleton_hardware check_joint_mapping)
install(FILES scripts/check_joint_mapping.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME check_joint_mapping
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
              GROUP_READ GROUP_EXECUTE
              WORLD_READ WORLD_EXECUTE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_hardware_interface test/test_hardware_interface.cpp)
  target_include_directories(test_hardware_interface PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_compile_features(test_hardware_interface PUBLIC cxx_std_17)
  target_link_libraries(test_hardware_interface ${PROJECT_NAME})
  ament_target_dependencies(test_hardware_interface
    hardware_interface)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  yaml_cpp_vendor
)

ament_package()
