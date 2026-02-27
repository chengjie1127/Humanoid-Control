cmake_minimum_required(VERSION 3.10)
project(humanoid_wbc)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  log
  log_setup
)
find_package(humanoid_interface REQUIRED)
find_package(qpoases_catkin REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${pinocchio_INCLUDE_DIRS}
)

link_directories(
        ${pinocchio_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME} SHARED
        src/HoQp.cpp
        src/WbcBase.cpp
        src/HierarchicalWbc.cpp
        src/WeightedWbc.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${pinocchio_LIBRARIES}
        ${qpoases_catkin_LIBRARIES}
        )
target_include_directories(${PROJECT_NAME} PUBLIC ${qpoases_catkin_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME} humanoid_interface)

# OCS2_CXX_FLAGS usually comes from ocs2_core or similar, we can safely just use target_compile_options if needed
# target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(humanoid_interface qpoases_catkin)

install(TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.h"
        )

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
