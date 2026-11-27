cmake_minimum_required(VERSION 3.16)
project(go2_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

add_executable(jie_pcd_octomap_node src/jie_pcd_octomap_node.cpp)
ament_target_dependencies(jie_pcd_octomap_node rclcpp octomap_msgs sensor_msgs visualization_msgs)
target_link_libraries(jie_pcd_octomap_node octomap)
install(TARGETS jie_pcd_octomap_node DESTINATION lib/${PROJECT_NAME})

qt5_wrap_cpp(MOC_GOAL3D_TOOL_FILES include/go2_bringup/Goal3DTool.hpp)
add_library(go2_bringup_goal3d_tool SHARED src/Goal3DTool.cpp ${MOC_GOAL3D_TOOL_FILES})
ament_target_dependencies(go2_bringup_goal3d_tool
  rclcpp
  geometry_msgs
  rviz_common
  rviz_rendering
  pluginlib
)
target_include_directories(go2_bringup_goal3d_tool PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(go2_bringup_goal3d_tool
  Qt5::Core
  Qt5::Widgets
)
install(TARGETS go2_bringup_goal3d_tool
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
pluginlib_export_plugin_description_file(rviz_common rviz_plugins.xml)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)

install(PROGRAMS scripts/jie_go2_navigation.py scripts/obstacle_height_filter.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
