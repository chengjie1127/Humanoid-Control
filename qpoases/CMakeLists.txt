cmake_minimum_required(VERSION 3.14)
project(qpoases)

find_package(ament_cmake REQUIRED)

include(FetchContent)

set(QPOASES_DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/download)
set(QPOASES_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/build)

set(BUILD_SHARED_LIBS ON CACHE STRING "Build shared libraries" FORCE)
set(QPOASES_BUILD_EXAMPLES OFF CACHE BOOL "Examples disable")

FetchContent_Declare(qpoasesDownload
        GIT_REPOSITORY https://github.com/coin-or/qpOASES
        GIT_TAG 268b2f2659604df27c82aa6e32aeddb8c1d5cc7f
        )
FetchContent_MakeAvailable(qpoasesDownload)

install(TARGETS qpOASES
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(DIRECTORY ${qpoasesdownload_SOURCE_DIR}/include/
        DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(qpOASES)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
