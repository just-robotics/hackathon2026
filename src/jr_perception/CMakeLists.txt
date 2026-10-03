cmake_minimum_required(VERSION 3.8)
project(jr_perception)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    jr_perception/robot_detector.py
    jr_perception/detector_eval.py
    jr_perception/record_background.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
