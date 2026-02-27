cmake_minimum_required(VERSION 3.10)
project(humanoid_common)

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
        DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies()
ament_package()
