cmake_minimum_required(VERSION 3.14)
project(qpoases_colcon)

find_package(ament_cmake REQUIRED)

include(FetchContent)

# Build settings
set(BUILD_SHARED_LIBS ON CACHE BOOL "Build shared libraries" FORCE)
set(QPOASES_BUILD_EXAMPLES OFF CACHE BOOL "Examples disable")

# Download & build source
FetchContent_Declare(qpoases
        GIT_REPOSITORY https://github.com/coin-or/qpOASES
        GIT_TAG 268b2f2659604df27c82aa6e32aeddb8c1d5cc7f
        UPDATE_COMMAND ""
    SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/qpoases-src
    BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/qpoases-build
        )
FetchContent_MakeAvailable(qpoases)

target_include_directories(qpOASES PUBLIC
    $<BUILD_INTERFACE:${qpoases_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS qpOASES
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY ${qpoases_SOURCE_DIR}/include/
    DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(qpOASES)
ament_export_include_directories(include)

ament_package()
