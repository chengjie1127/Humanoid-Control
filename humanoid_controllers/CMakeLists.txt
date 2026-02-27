cmake_minimum_required(VERSION 3.8)
project(humanoid_controllers)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(AMENT_PACKAGE_DEPENDENCIES
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  ocs2_msgs
  tf2_ros
  tf2_geometry_msgs
  robot_state_publisher
  urdf
  humanoid_common
  humanoid_interface
  humanoid_wbc
  humanoid_estimation
  humanoid_dummy
  ocs2_self_collision_visualization
  angles
)

find_package(ament_cmake REQUIRED)
foreach(dep ${AMENT_PACKAGE_DEPENDENCIES})
  find_package(${dep} REQUIRED)
endforeach()

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  log
  log_setup
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

include_directories(
  include
  ${pinocchio_INCLUDE_DIRS}
)

link_directories(
  ${pinocchio_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/humanoidController.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${pinocchio_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME} ${AMENT_PACKAGE_DEPENDENCIES})

# Target trajectories publish node
add_executable(humanoid_target_trajectories_publisher
  src/TargetTrajectoriesPublisher.cpp
)
target_link_libraries(humanoid_target_trajectories_publisher
  ${PROJECT_NAME}
)
ament_target_dependencies(humanoid_target_trajectories_publisher ${AMENT_PACKAGE_DEPENDENCIES})

# Cheat Controller Node
add_executable(cheat_controller_node
  src/CheatControllerNode.cpp
)
target_link_libraries(cheat_controller_node
  ${PROJECT_NAME}
)
ament_target_dependencies(cheat_controller_node ${AMENT_PACKAGE_DEPENDENCIES})

# Normal Controller Node
add_executable(normal_controller_node
  src/NormalControllerNode.cpp
)
target_link_libraries(normal_controller_node
  ${PROJECT_NAME}
)
ament_target_dependencies(normal_controller_node ${AMENT_PACKAGE_DEPENDENCIES})

# Joy Controller Node
add_executable(joy_controller_node
  src/JoyControllerNode.cpp
)
target_link_libraries(joy_controller_node
  ${PROJECT_NAME}
)
ament_target_dependencies(joy_controller_node ${AMENT_PACKAGE_DEPENDENCIES})

# Install
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  humanoid_target_trajectories_publisher
  cheat_controller_node
  normal_controller_node
  joy_controller_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()