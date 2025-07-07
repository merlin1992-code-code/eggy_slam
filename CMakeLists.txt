cmake_minimum_required(VERSION 3.10)
project(EGGY LANGUAGES C CXX)

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
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(jsoncpp CONFIG REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(TBB REQUIRED)

add_subdirectory(thirdparty)
add_definitions("-D SOPHUS_USE_BASIC_LOGGING")

include_directories(thirdparty/Sophus
    ${PCL_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/fastlio/include
    ${PROJECT_SOURCE_DIR}/fastlio/map_builder
    ${PROJECT_SOURCE_DIR}/m-detector/include
    ${PROJECT_SOURCE_DIR}/m-detector/include/cluster_predict)

# fastlio
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

# m-detector
set(M_DETECTOR_SRC
    m-detector/DynObjCluster.cpp
    m-detector/DynObjFilter.cpp
    m-detector/dyn_node.cpp
)

add_executable(eggy main.cpp ${LIO_SRC_LIST} ${M_DETECTOR_SRC})

target_compile_definitions(eggy PUBLIC SOPHUS_USE_BASIC_LOGGING)

target_link_libraries(eggy PUBLIC
    ${TBB_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    rosbag
    Eigen3::Eigen
    TBB::tbb
    pthread
    PROJ::proj
    yaml-cpp::yaml-cpp
    nlohmann_json::nlohmann_json
    glm::glm
    imgui
    tsl::robin_map
)

# tools
#bag
set(BAG_SRC
    tools/toRosbag/toRosbag.cpp
)
add_executable(ToRosbag tools/toRosbag/main.cpp ${BAG_SRC})
target_include_directories(ToRosbag PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(ToRosbag
    rosbag
    yaml-cpp::yaml-cpp
    nlohmann_json::nlohmann_json
    ${PCL_LIBRARIES}
)
#stitch
set(STITCH_SRC
    tools/stitch/stitch.cpp
)
add_executable(stitch
    ${STITCH_SRC}
)
target_link_libraries(stitch
    yaml-cpp::yaml-cpp
    nlohmann_json::nlohmann_json
    Eigen3::Eigen
    ${PCL_LIBRARIES}
)