cmake_minimum_required(VERSION 3.8)
project(example)

set(CMAKE_CXX_FLAGS "-std=c++14 -march=native -O3")
set(CMAKE_BUILD_TYPE "Release")

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
set(BOOST_COMPONENTS iostreams filesystem)
find_package(Boost REQUIRED COMPONENTS ${BOOST_COMPONENTS})
find_package(Ceres REQUIRED)
find_package(Protobuf REQUIRED)

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${YAML_CPP_INCLUDE_DIRS}
    ${OPENCV_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/../install/include
    ${CMAKE_BINARY_DIR}
    ${Boost_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
)

add_executable(demo demo.cpp)
target_link_libraries(demo
    glog
    ${YAML_CPP_LIBRARIES}
    ${OpenCV_LIBS}
    ${CMAKE_SOURCE_DIR}/../install/lib/liboccmap.a
    ${CMAKE_SOURCE_DIR}/../install/lib/libproto.a
    ${Boost_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    ${CERES_LIBRARIES}
    ${Protobuf_LIBRARIES}
)