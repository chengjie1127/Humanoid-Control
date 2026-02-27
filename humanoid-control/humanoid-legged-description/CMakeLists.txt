cmake_minimum_required(VERSION 3.8)

project(humanoid_legged_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes mjcf urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
