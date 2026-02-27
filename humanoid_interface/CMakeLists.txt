cmake_minimum_required(VERSION 3.8)
project(humanoid_interface)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(AMENT_PACKAGE_DEPENDENCIES
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
  ocs2_self_collision
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

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

# Resolve for the package path at compile time.
configure_file (
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in"
  "${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/package_path.h" @ONLY
)

set(FLAGS
  # ${OCS2_CXX_FLAGS}
  ${pinocchio_CFLAGS_OTHER}
  -Wno-ignored-attributes
  -Wno-invalid-partial-specialization   # to silence warning with unsupported Eigen Tensor
  -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
  -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
)

include_directories(
  include
  ${pinocchio_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(
  ${pinocchio_LIBRARY_DIRS}
)

# Legged robot interface library
add_library(${PROJECT_NAME} SHARED
  src/common/ModelSettings.cpp
  src/dynamics/HumanoidDynamicsAD.cpp
  src/constraint/EndEffectorLinearConstraint.cpp
  src/constraint/FrictionConeConstraint.cpp
  src/constraint/ZeroForceConstraint.cpp
  src/constraint/NormalVelocityConstraintCppAd.cpp
  src/constraint/ZeroVelocityConstraintCppAd.cpp
  src/constraint/FootRollContraint.cpp
  src/initialization/HumanoidInitializer.cpp
  src/reference_manager/SwitchedModelReferenceManager.cpp
  src/foot_planner/CubicSpline.cpp
  src/foot_planner/SplineCpg.cpp
  src/foot_planner/SwingTrajectoryPlanner.cpp
  src/gait/Gait.cpp
  src/gait/GaitSchedule.cpp
  src/gait/LegLogic.cpp
  src/gait/ModeSequenceTemplate.cpp
  src/HumanoidInterface.cpp
  src/HumanoidPreComputation.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${Boost_LIBRARIES}
  ${pinocchio_LIBRARIES}
  dl
)
foreach(dep ${AMENT_PACKAGE_DEPENDENCIES})
  if (NOT ${dep} MATCHES "ocs2_robotic_assets")
    target_link_libraries(${PROJECT_NAME} PUBLIC ${dep}::${dep})
  endif()
endforeach()

target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
foreach(dep ${AMENT_PACKAGE_DEPENDENCIES})
  if (NOT ${dep} MATCHES "ocs2_robotic_assets")
    ament_export_dependencies(${dep})
  endif()
endforeach()
ament_export_dependencies(Boost)
ament_export_dependencies(Eigen3)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
