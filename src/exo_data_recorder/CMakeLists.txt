cmake_minimum_required(VERSION 3.8)
project(exo_data_recorder)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/check_topics.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME check_topics
)

install(PROGRAMS scripts/verify_bag.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME verify_bag
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
