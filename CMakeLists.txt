cmake_minimum_required(VERSION 3.10)
project(FastLIO LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

if(NOT CXX_MARCH)
    set(CXX_MARCH native)
endif()


find_package(PCL REQUIRED)
find_package(PROJ REQUIRED)
add_subdirectory(thirdparty)
add_definitions("-D SOPHUS_USE_BASIC_LOGGING")

include_directories(thirdparty/Sophus
    ${PCL_INCLUDE_DIRS})

#fastlio
set(LIO_SRC_LIST
    fastlio/map_builder/commons.cpp
    fastlio/map_builder/ieskf.cpp
    fastlio/map_builder/imu_processor.cpp
    fastlio/map_builder/ikd_Tree.cpp
    fastlio/map_builder/lidar_processor.cpp
    fastlio/map_builder/map_builder.cpp
    fastlio/utils.cpp
    fastlio/lio_node.cpp
)

#m-detector
set(M_DETECTOR_SRC
    m-detector/DynObjCluster.cpp
    m-detector/DynObjFilter.cpp
    m-detector/dyr_node.cpp
)

add_executable(eggy main.cpp ${LIO_SRC_LIST} ${M_DETECTOR_SRC})

target_compile_definitions(eggy PUBLIC SOPHUS_USE_BASIC_LOGGING)

target_link_libraries(eggy PUBLIC
    yaml-cpp::yaml-cpp
    Eigen3::Eigen
    ${TBB_LIBRARIES}
    ${PCL_LIBRARIES}
    rosbag
    pthread
    imgui
    glm::glm
    tsl::robin_map
    TBB::tbb
    PROJ::PROJ
)