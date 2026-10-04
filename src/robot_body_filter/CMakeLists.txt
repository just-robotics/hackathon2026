cmake_minimum_required(VERSION 3.8)
project(robot_body_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(robot_body_filter_node
  src/crop_box.cpp
  src/robot_body_filter.cpp
  src/robot_body_filter_node.cpp
)

target_include_directories(robot_body_filter_node
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(robot_body_filter_node
  Eigen3::Eigen
)

ament_target_dependencies(robot_body_filter_node
  geometry_msgs
  rclcpp
  sensor_msgs
  tf2_eigen
  tf2_ros
  tf2_sensor_msgs
  visualization_msgs
)

install(DIRECTORY
    src
    DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
    robot_body_filter_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
