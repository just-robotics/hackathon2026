cmake_minimum_required(VERSION 3.8)
project(jr_referee)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS jr_referee/duel_referee.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
