cmake_minimum_required(VERSION 3.8)
project(octo_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(jie_map_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(octomap REQUIRED)

add_executable(jie_path_node src/jie_path_node.cpp)
ament_target_dependencies(jie_path_node
  geometry_msgs jie_map_msgs nav_msgs octomap_msgs rclcpp sensor_msgs visualization_msgs)
target_link_libraries(jie_path_node octomap)

add_executable(go2_path_follower src/d1_controller.cpp)
target_include_directories(go2_path_follower PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(go2_path_follower
  geometry_msgs nav_msgs rclcpp std_msgs tf2 tf2_geometry_msgs tf2_ros visualization_msgs)
target_link_libraries(go2_path_follower ${OpenCV_LIBS})

install(TARGETS jie_path_node go2_path_follower
  DESTINATION lib/${PROJECT_NAME})
install(FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages)
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

ament_package()
