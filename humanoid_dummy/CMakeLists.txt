cmake_minimum_required(VERSION 3.8)
project(humanoid_dummy)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(AMENT_PACKAGE_DEPENDENCIES
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  urdf
  robot_state_publisher
  ocs2_core
  ocs2_oc
  ocs2_ddp
  ocs2_mpc
  ocs2_sqp
  ocs2_ipm
  ocs2_robotic_tools
  ocs2_pinocchio_interface
  ocs2_centroidal_model
  ocs2_robotic_assets
  ocs2_msgs
  ocs2_ros_interfaces
  humanoid_interface
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

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(pinocchio REQUIRED)


include_directories(
  include
  ${pinocchio_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(
  ${pinocchio_LIBRARY_DIRS}
)

# Main library
add_library(${PROJECT_NAME} SHARED
  src/gait/GaitKeyboardPublisher.cpp
  src/gait/GaitReceiver.cpp
  src/visualization/HumanoidVisualizer.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${pinocchio_LIBRARIES}
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} ${AMENT_PACKAGE_DEPENDENCIES})
# target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

# DDP-MPC node for legged robot
add_executable(humanoid_ddp_mpc
  src/HumanoidDdpMpcNode.cpp
)
target_link_libraries(humanoid_ddp_mpc
  ${PROJECT_NAME}
)
ament_target_dependencies(humanoid_ddp_mpc ${AMENT_PACKAGE_DEPENDENCIES})
# target_compile_options(humanoid_ddp_mpc PRIVATE ${OCS2_CXX_FLAGS})

## SQP-MPC node for legged robot
add_executable(humanoid_sqp_mpc
  src/HumanoidSqpMpcNode.cpp
)
target_link_libraries(humanoid_sqp_mpc
  ${PROJECT_NAME}
)
ament_target_dependencies(humanoid_sqp_mpc ${AMENT_PACKAGE_DEPENDENCIES})
# target_compile_options(humanoid_sqp_mpc  PRIVATE ${OCS2_CXX_FLAGS})

## IPM-MPC node for legged robot
add_executable(humanoid_ipm_mpc
  src/HumanoidIpmMpcNode.cpp
)
target_link_libraries(humanoid_ipm_mpc
  ${PROJECT_NAME}
)
ament_target_dependencies(humanoid_ipm_mpc ${AMENT_PACKAGE_DEPENDENCIES})
# target_compile_options(humanoid_ipm_mpc  PRIVATE ${OCS2_CXX_FLAGS})

# Dummy node
add_executable(humanoid_dummy_node
  src/HumanoidDummyNode.cpp
)
target_link_libraries(humanoid_dummy_node
  ${PROJECT_NAME}
)
ament_target_dependencies(humanoid_dummy_node ${AMENT_PACKAGE_DEPENDENCIES})
# target_compile_options(humanoid_dummy_node PRIVATE ${OCS2_CXX_FLAGS})

# Keyboard command node for target trajectories
add_executable(humanoid_target
  src/HumanoidPoseCommandNode.cpp
)
target_link_libraries(humanoid_target
  ${PROJECT_NAME}
)
ament_target_dependencies(humanoid_target ${AMENT_PACKAGE_DEPENDENCIES})
# target_compile_options(humanoid_target PRIVATE ${OCS2_CXX_FLAGS})

# Keyboard command node for gait
add_executable(humanoid_gait_command
  src/HumanoidGaitCommandNode.cpp
)
target_link_libraries(humanoid_gait_command
  ${PROJECT_NAME}
)
ament_target_dependencies(humanoid_gait_command ${AMENT_PACKAGE_DEPENDENCIES})
# target_compile_options(humanoid_gait_command PRIVATE ${OCS2_CXX_FLAGS})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
    humanoid_ddp_mpc
    humanoid_sqp_mpc
    humanoid_ipm_mpc
    humanoid_dummy_node
    humanoid_target
    humanoid_gait_command
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${AMENT_PACKAGE_DEPENDENCIES})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
