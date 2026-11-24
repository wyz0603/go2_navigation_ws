cmake_minimum_required(VERSION 3.16)
project(go2_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(jie_pcd_octomap_node src/jie_pcd_octomap_node.cpp)
ament_target_dependencies(jie_pcd_octomap_node rclcpp octomap_msgs sensor_msgs visualization_msgs)
target_link_libraries(jie_pcd_octomap_node octomap)
install(TARGETS jie_pcd_octomap_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)

install(PROGRAMS scripts/jie_go2_navigation.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
