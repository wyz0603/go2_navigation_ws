cmake_minimum_required(VERSION 3.16)
project(go2_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  DESTINATION share/
)

ament_package()
