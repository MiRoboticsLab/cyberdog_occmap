cmake_minimum_required(VERSION 3.8)
project(occmap)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/install)

find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
set(BOOST_COMPONENTS iostreams filesystem)
find_package(Boost REQUIRED COMPONENTS ${BOOST_COMPONENTS})
find_package(yaml-cpp REQUIRED)
find_package(Ceres REQUIRED)


include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/common
  ${CMAKE_BINARY_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS})

add_subdirectory(proto)

add_library(${PROJECT_NAME} src/grid_mapper.cpp
                            src/sensor.cpp
                            src/proto_io.cpp
                            src/rigid_transform.cpp
                            src/grid_2d.cpp
                            src/grid_inserter_2d.cpp
                            src/probability_value.cpp
                            src/ray_to_pixel_mask.cpp
                            src/value_conversion_tables.cpp
                            src/image.cpp
                            src/occupied_space_cost_function_2d.cpp
                            src/ceres_scan_matching_2d.cpp
                            src/submap_2d.cpp
                            src/map_beauti.cpp)

target_link_libraries(${PROJECT_NAME}
                      proto
                      glog
                      yaml-cpp
                      ${Boost_LIBRARIES}
                      ${CMAKE_THREAD_LIBS_INIT}
                      ${OpenCV_LIBS}
                      ${YAML_CPP_LIBRARIES}
                      ${CERES_LIBRARIES})

install(TARGETS
  ${PROJECT_NAME}
  LIBRARY DESTINATION lib)

install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/include/map2d
  ${PROJECT_SOURCE_DIR}/include/occ_gridmap
  ${PROJECT_SOURCE_DIR}/include/scan_matching
  DESTINATION include)

install(FILES
  ${CMAKE_BINARY_DIR}/proto/transform.pb.h
  ${CMAKE_BINARY_DIR}/proto/range_data.pb.h
  DESTINATION include/proto)