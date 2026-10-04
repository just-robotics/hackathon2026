cmake_minimum_required(VERSION 3.8)
project(dbscan_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(dbscan_filter_node
  src/dbscan.cpp
  src/dbscan_filter_node.cpp
)

target_include_directories(dbscan_filter_node
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(dbscan_filter_node
  Eigen3::Eigen
)

ament_target_dependencies(dbscan_filter_node
  rclcpp
  sensor_msgs
)

install(DIRECTORY
    config
    launch
    DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
    dbscan_filter_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
