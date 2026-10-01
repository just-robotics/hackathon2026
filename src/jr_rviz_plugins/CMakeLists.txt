cmake_minimum_required(VERSION 3.8)
project(jr_rviz_plugins)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(jr_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

# Панель наследует QObject и объявляет слоты, поэтому нужен moc.
set(CMAKE_AUTOMOC ON)

add_library(${PROJECT_NAME} SHARED
  src/duel_status_panel.cpp
  include/jr_rviz_plugins/duel_status_panel.hpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  rviz_common::rviz_common
  Qt5::Widgets
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  pluginlib
  jr_msgs
)

# Без этого RViz не найдёт библиотеку через pluginlib.
target_compile_definitions(${PROJECT_NAME} PRIVATE "JR_RVIZ_PLUGINS_BUILDING_LIBRARY")

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(FILES plugins_description.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_package()
