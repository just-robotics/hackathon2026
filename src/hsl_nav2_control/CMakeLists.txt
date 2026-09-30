cmake_minimum_required(VERSION 3.8)
project(hsl_nav2_control)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
foreach(dependency rclcpp rclcpp_lifecycle nav2_mppi_controller nav2_controller
        nav2_costmap_2d nav2_util tf2 tf2_ros tf2_geometry_msgs geometry_msgs
        nav_msgs sensor_msgs std_msgs hsl_interfaces)
  find_package(${dependency} REQUIRED)
endforeach()
add_executable(native_mppi src/native_mppi.cpp)
ament_target_dependencies(native_mppi rclcpp rclcpp_lifecycle nav2_mppi_controller
  nav2_controller nav2_costmap_2d nav2_util tf2 tf2_ros tf2_geometry_msgs
  geometry_msgs nav_msgs sensor_msgs std_msgs hsl_interfaces)
target_compile_options(native_mppi PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS native_mppi DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_package()
