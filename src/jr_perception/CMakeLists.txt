cmake_minimum_required(VERSION 3.8)
project(jr_perception)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(opponent_core src/opponent_core.cpp)
target_compile_features(opponent_core PUBLIC cxx_std_17)
target_include_directories(opponent_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(opponent_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(opponent_detector_cpp src/opponent_node.cpp)
target_compile_features(opponent_detector_cpp PUBLIC cxx_std_17)
target_compile_options(opponent_detector_cpp PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(opponent_detector_cpp opponent_core)
ament_target_dependencies(opponent_detector_cpp
  rclcpp builtin_interfaces geometry_msgs nav_msgs sensor_msgs std_msgs
  tf2 tf2_ros visualization_msgs)

install(TARGETS opponent_core opponent_detector_cpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    jr_perception/robot_detector.py
    jr_perception/detector_eval.py
    jr_perception/record_background.py
    jr_perception/initial_pose.py
    jr_perception/pose_relay.py
    jr_perception/livox_custom.py
    jr_perception/fastlio_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
