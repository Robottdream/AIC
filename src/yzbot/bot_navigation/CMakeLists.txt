cmake_minimum_required(VERSION 3.5)
project(bot_navigation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(dwb_core REQUIRED)
find_package(dwb_plugins REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_path(NLOHMANN_JSON_INCLUDE_DIR nlohmann/json.hpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
add_library(aic_predictive_critic SHARED src/predictive_obstacle_critic.cpp src/known_trajectory_layer.cpp src/fine_omni_trajectory_generator.cpp src/path_follow_critic.cpp)
target_include_directories(aic_predictive_critic PRIVATE ${NLOHMANN_JSON_INCLUDE_DIR})
ament_target_dependencies(aic_predictive_critic dwb_core dwb_plugins nav2_costmap_2d pluginlib std_msgs)
pluginlib_export_plugin_description_file(dwb_core predictive_critic.xml)
pluginlib_export_plugin_description_file(nav2_costmap_2d known_trajectory_layer.xml)
install(TARGETS aic_predictive_critic LIBRARY DESTINATION lib)
install(
  DIRECTORY launch param maps config rviz
  DESTINATION share/${PROJECT_NAME}
)
install(
  PROGRAMS scripts/bright_costmap.py scripts/scan_velocity_guard.py scripts/mecanum_io.py scripts/known_obstacle_forecaster.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES scripts/dynamic_obstacle_tracker.py DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
