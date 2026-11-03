cmake_minimum_required(VERSION 3.16)
project(go2_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  meshes
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
