cmake_minimum_required(VERSION 3.10)
project(humanoid_controllers)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(humanoid_common REQUIRED)
find_package(humanoid_interface REQUIRED)
find_package(humanoid_wbc REQUIRED)
find_package(humanoid_estimation REQUIRED)
find_package(humanoid_dummy REQUIRED)
find_package(ocs2_self_collision_visualization REQUIRED)
find_package(angles REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ocs2_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

###########
## Build ##
###########

include_directories(
        include
        ${pinocchio_INCLUDE_DIRS}
)

link_directories(
        ${pinocchio_LIBRARY_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
        src/humanoidController.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${pinocchio_LIBRARIES}
        )
ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        controller_interface
        humanoid_common
        humanoid_interface
        humanoid_wbc
        humanoid_estimation
        humanoid_dummy
        ocs2_self_collision_visualization
        angles
        std_msgs
        sensor_msgs
        geometry_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
        pluginlib
        ocs2_msgs
)

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})


# Target trajectories publish node
add_executable(humanoid_target_trajectories_publisher
        src/TargetTrajectoriesPublisher.cpp
        )
add_dependencies(humanoid_target_trajectories_publisher
        ${PROJECT_NAME}
        )
target_link_libraries(humanoid_target_trajectories_publisher
        ${PROJECT_NAME}
        )
ament_target_dependencies(humanoid_target_trajectories_publisher rclcpp)
target_compile_options(humanoid_target_trajectories_publisher PRIVATE ${OCS2_CXX_FLAGS})

# Cheat Controller Node
add_executable(cheat_controller_node
        src/CheatControllerNode.cpp
        )
add_dependencies(cheat_controller_node
        ${PROJECT_NAME}
        )
target_link_libraries(cheat_controller_node
        ${PROJECT_NAME}
        )
ament_target_dependencies(cheat_controller_node rclcpp)
target_compile_options(cheat_controller_node PRIVATE ${OCS2_CXX_FLAGS})

# Normal Controller Node
add_executable(normal_controller_node
        src/NormalControllerNode.cpp
        )
add_dependencies(normal_controller_node
        ${PROJECT_NAME}
        )
target_link_libraries(normal_controller_node
        ${PROJECT_NAME}
        )
ament_target_dependencies(normal_controller_node rclcpp)
target_compile_options(normal_controller_node PRIVATE ${OCS2_CXX_FLAGS})

# Joy Controller Node
add_executable(joy_controller_node
        src/JoyControllerNode.cpp
        )
add_dependencies(joy_controller_node
        ${PROJECT_NAME}
        )
target_link_libraries(joy_controller_node
        ${PROJECT_NAME}
        )
ament_target_dependencies(joy_controller_node rclcpp)
target_compile_options(joy_controller_node PRIVATE ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME} humanoid_target_trajectories_publisher
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        )

install(TARGETS humanoid_target_trajectories_publisher cheat_controller_node normal_controller_node joy_controller_node
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.h"
        )

install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
        rclcpp
        controller_interface
        humanoid_common
        humanoid_interface
        humanoid_wbc
        humanoid_estimation
        humanoid_dummy
        ocs2_self_collision_visualization
        angles
)
ament_export_include_directories(include)

ament_package()