cmake_minimum_required(VERSION 3.17)
project(PointCloud_Segment)

set(CMAKE_CXX_STANDARD 17)

find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
add_definitions(${PCL_DEFINITIONS})
include(cmake/glog.cmake)

find_package(Boost REQUIRED COMPONENTS regex thread system filesystem)
add_definitions(-DBOOST_NO_CXX11_SCOPED_ENUMS)

include_directories(
        ${Boost_INCLUDE_DIRS}
)

include(cmake/global_definition.cmake)
include_directories(
        include
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${PCL_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDEDIR}
)

add_library(${PROJECT_NAME}
        SHARED
        src/planeSeg.cpp
        src/config.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${PCL_LIBRARY_DIRS}
        ${PCL_LIBRARIES}
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        ${GLOG_LIBRARIES}
        )

add_executable(plane_segmentation example/plane_seg.cpp)
target_link_libraries(plane_segmentation ${PROJECT_NAME})