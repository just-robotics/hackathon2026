cmake_minimum_required(VERSION 3.8)
project(jr_planning)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    jr_planning/potential_field_planner.py
    jr_planning/game_mpc_planner.py
    jr_planning/attack_strategy.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
