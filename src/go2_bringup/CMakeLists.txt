cmake_minimum_required(VERSION 3.16)
project(go2_bringup)

find_package(ament_cmake REQUIRED)
find_package(nav2_controller REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(go2_height_aware_goal_checker SHARED src/height_aware_goal_checker.cpp)
target_compile_features(go2_height_aware_goal_checker PUBLIC cxx_std_17)
ament_target_dependencies(go2_height_aware_goal_checker
  nav2_controller nav2_core pluginlib rclcpp rclcpp_lifecycle geometry_msgs)
pluginlib_export_plugin_description_file(nav2_core goal_checker_plugins.xml)
install(TARGETS go2_height_aware_goal_checker LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_height_aware_goal_checker tests/test_height_aware_goal_checker.cpp)
  ament_target_dependencies(test_height_aware_goal_checker
    nav2_core pluginlib rclcpp rclcpp_lifecycle geometry_msgs)
endif()

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/obstacle_height_filter.py scripts/ground_goal_adapter.py DESTINATION lib/${PROJECT_NAME})

ament_package()
