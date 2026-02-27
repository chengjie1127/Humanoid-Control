cmake_minimum_required(VERSION 3.10)
project(humanoid_wbc)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(humanoid_interface REQUIRED)
find_package(qpoases_colcon REQUIRED)
find_package(pinocchio REQUIRED)
find_package(ament_cmake_gtest QUIET)
find_package(Boost REQUIRED COMPONENTS system filesystem log log_setup)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

###########
## Build ##
###########

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${pinocchio_INCLUDE_DIRS}
)

link_directories(
        ${pinocchio_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME}
        src/HoQp.cpp
        src/WbcBase.cpp
        src/HierarchicalWbc.cpp
        src/WeightedWbc.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${pinocchio_LIBRARIES}
        )
ament_target_dependencies(${PROJECT_NAME}
        humanoid_interface
        qpoases_colcon
)

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}
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

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.h"
        )

#############
## Testing ##
#############
if (BUILD_TESTING AND ament_cmake_gtest_FOUND)
    ament_add_gtest(${PROJECT_NAME}_test
            test/HoQp_test.cpp
    )

    target_link_libraries(${PROJECT_NAME}_test
            ${PROJECT_NAME}
            ${pinocchio_LIBRARIES}
            gtest_main
            )

    target_compile_options(${PROJECT_NAME}_test PUBLIC ${FLAGS})
endif ()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(humanoid_interface qpoases_colcon)
ament_export_include_directories(include)

ament_package()
