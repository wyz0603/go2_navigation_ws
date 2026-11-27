cmake_minimum_required(VERSION 3.22)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -std=c++17 -g")
project(scan_planner)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include)

include_directories(${Boost_INCLUDE_DIRS})

include_directories(./planner/bspline_opt/include)
include_directories(./planner/path_searching/include)
include_directories(./planner/plan_env/include)
include_directories(./planner/plan_manage/include/plan_manage)
include_directories(./planner/plan_manage/include)
include_directories(./planner/include)

add_subdirectory(planner/plan_manage)
add_subdirectory(planner/bspline_opt)
add_subdirectory(planner/path_searching)
add_subdirectory(planner/plan_env)

# 编译节点
add_executable(motion_plan src/trajectory_publisher.cpp)
ament_target_dependencies(
  motion_plan
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  sensor_msgs
  std_msgs
  visualization_msgs
)

target_link_libraries(
  motion_plan
  bspline_opt
  path_searching
  local_map
  scan_planner
  ${Boost_LIBRARIES}
)

add_executable(scan_local_planner src/scan_local_planner.cpp)
ament_target_dependencies(scan_local_planner rclcpp nav_msgs geometry_msgs sensor_msgs tf2 tf2_ros)
target_link_libraries(scan_local_planner bspline_opt path_searching local_map scan_planner ${Boost_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_scan_core tests/test_scan_core.cpp)
  target_link_libraries(test_scan_core scan_planner bspline_opt path_searching local_map ${Boost_LIBRARIES})
  ament_target_dependencies(test_scan_core rclcpp geometry_msgs)
endif()

# 安装节点
install(TARGETS
  motion_plan
  scan_local_planner
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)

install(FILES
  rviz.rviz
  DESTINATION share/${PROJECT_NAME}/rviz)

ament_package()
