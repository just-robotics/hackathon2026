cmake_minimum_required(VERSION 3.8)
project(sim_kobuki)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config description launch meshes worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
