cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

project(FastLIO LANGUAGES C CXX)

if(NOT CXX_MARCH)
    set(CXX_MARCH native)
endif()

find_package(PCL REQUIRED)

add_subdirectory(thirdparty)

add_definitions("-D SOPHUS_USE_BASIC_LOGGING")
include_directories(thirdparty/Sophus
    ${PCL_INCLUDE_DIRS})

set(SRC_LIST fastlio/src/map_builder/commons.cpp
    fastlio/src/map_builder/ieskf.cpp
    fastlio/src/map_builder/imu_processor.cpp
    fastlio/src/map_builder/ikd_Tree.cpp
    fastlio/src/map_builder/lidar_processor.cpp
    fastlio/src/map_builder/map_builder.cpp
    fastlio/src/utils.cpp)

add_executable(lio_node fastlio/src/lio_node.cpp ${SRC_LIST})
target_link_libraries(lio_node PUBLIC
    yaml-cpp::yaml-cpp
    Eigen3::Eigen
    ${TBB_LIBRARIES}
    ${PCL_LIBRARIES}
    rosbag
    pthread
    imgui glm::glm
    PRIVATE tsl::robin_map
    TBB::tbb
)
