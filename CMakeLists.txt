cmake_minimum_required(VERSION 3.8)
project(cyberdog_occmap)

execute_process(COMMAND bash ${CMAKE_SOURCE_DIR}/3rdparty/occmap/install.sh
                WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/3rdparty/occmap
                OUTPUT_VARIABLE output_var
                RESULT_VARIABLE result_var
                ERROR_VARIABLE error_var)
message(STATUS "occmap result: ${result_var}")
message(STATUS "occmap output: ${output_var}")
message(STATUS "occmap error: ${error_var}")

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "-std=c++14 -march=native -O3")
set(CMAKE_BUILD_TYPE "Release")
set(LOG_OUT "Release")
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(message_filters REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cyberdog_visions_interfaces REQUIRED)
find_package(cyberdog_common REQUIRED)
find_package(OpenCV 4 REQUIRED)
set(BOOST_COMPONENTS iostreams filesystem)
find_package(Boost REQUIRED COMPONENTS ${BOOST_COMPONENTS})
find_package(yaml-cpp REQUIRED)
find_package(Ceres REQUIRED)
find_package(Protobuf REQUIRED)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/3rdparty/occmap/install/include
  ${CMAKE_BINARY_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS})

add_executable(create_gridmap_node src/create_gridmap.cpp)

ament_target_dependencies(create_gridmap_node
                          "rclcpp"
                          "message_filters"
                          "geometry_msgs"
                          "nav_msgs"
                          "sensor_msgs"
                          "std_msgs"
                          "std_srvs"
                          "cyberdog_visions_interfaces"
                          "cyberdog_common")

target_link_libraries(create_gridmap_node
                      glog
                      yaml-cpp
                      ${CMAKE_SOURCE_DIR}/3rdparty/occmap/install/lib/liboccmap.a
                      ${CMAKE_SOURCE_DIR}/3rdparty/occmap/install/lib/libproto.a
                      ${Boost_LIBRARIES}
                      ${CMAKE_THREAD_LIBS_INIT}
                      ${OpenCV_LIBS}
                      ${YAML_CPP_LIBRARIES}
                      ${CERES_LIBRARIES}
                      ${Protobuf_LIBRARIES})

add_executable(publish_gridmap_node src/publish_gridmap.cpp)

ament_target_dependencies(publish_gridmap_node
                          "rclcpp"
                          "std_msgs"
                          "nav_msgs"
                          "cyberdog_visions_interfaces"
                          "cyberdog_common")

target_link_libraries(publish_gridmap_node
                      glog
                      yaml-cpp
                      ${CMAKE_SOURCE_DIR}/3rdparty/occmap/install/lib/liboccmap.a
                      ${CMAKE_SOURCE_DIR}/3rdparty/occmap/install/lib/libproto.a
                      ${Boost_LIBRARIES}
                      ${CMAKE_THREAD_LIBS_INIT}
                      ${OpenCV_LIBS}
                      ${YAML_CPP_LIBRARIES}
                      ${CERES_LIBRARIES}
                      ${Protobuf_LIBRARIES})

install(TARGETS
  create_gridmap_node
  publish_gridmap_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch param DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
