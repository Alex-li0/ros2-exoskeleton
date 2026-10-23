cmake_minimum_required(VERSION 3.8)
project(cpg_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(terrain_adaptive_cpg REQUIRED)
find_package(exo_state_estimator REQUIRED)

add_executable(single_leg_cpg src/single_leg_cpg.cpp)
ament_target_dependencies(single_leg_cpg rclcpp sensor_msgs)

add_executable(legs_cpg src/legs_cpg.cpp)
target_compile_features(legs_cpg PRIVATE cxx_std_17)
target_include_directories(legs_cpg PRIVATE include)
ament_target_dependencies(legs_cpg rclcpp std_msgs sensor_msgs terrain_adaptive_cpg)


add_executable(adaptive_cpg_shadow src/adaptive_cpg_shadow_node.cpp)
target_compile_features(adaptive_cpg_shadow PRIVATE cxx_std_17)
target_include_directories(adaptive_cpg_shadow PRIVATE include)
ament_target_dependencies(adaptive_cpg_shadow rclcpp sensor_msgs std_msgs exo_state_estimator)

add_executable(adaptive_learning_probe scripts/adaptive_learning_probe.cpp)
target_compile_features(adaptive_learning_probe PRIVATE cxx_std_17)
target_include_directories(adaptive_learning_probe PRIVATE include)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_cpg_node test/test_cpg_node.py
    ENV CPG_BINARY=$<TARGET_FILE:legs_cpg> TIMEOUT 45)
  ament_add_pytest_test(test_adaptive_node test/test_adaptive_node.py
    ENV ADAPTIVE_CPG_BINARY=$<TARGET_FILE:adaptive_cpg_shadow>
        CPG_PRIOR_FILE=${CMAKE_CURRENT_SOURCE_DIR}/config/cpg.yaml TIMEOUT 60)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_adaptive_cpg test/test_adaptive_cpg.cpp)
  target_include_directories(test_adaptive_cpg PRIVATE include)
  target_compile_features(test_adaptive_cpg PRIVATE cxx_std_17)
  ament_add_gtest(test_reference_math test/test_reference_math.cpp)
  target_include_directories(test_reference_math PRIVATE include)
  target_compile_features(test_reference_math PRIVATE cxx_std_17)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  single_leg_cpg legs_cpg adaptive_cpg_shadow adaptive_learning_probe
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/cpg_optimizer/cpg_optimizer.py
  scripts/evaluate_adaptive_learning.py
  scripts/replay_adaptive_measurements.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
