cmake_minimum_required(VERSION 3.14.4)
project(mav_planning_rviz)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

cmake_policy(VERSION 3.16)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(planner_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(visualization_msgs REQUIRED)

set(mav_planning_rviz_plugins_headers_to_moc
  include/mav_planning_rviz/pose_widget.h
  include/mav_planning_rviz/planning_panel.h
  include/mav_planning_rviz/edit_button.h
)

add_library(${PROJECT_NAME} SHARED
  src/planning_panel.cpp
  src/pose_widget.cpp
  src/edit_button.cpp
  src/goal_marker.cpp
  ${mav_planning_rviz_plugins_headers_to_moc}
)

ament_target_dependencies(${PROJECT_NAME}
  grid_map_core
  grid_map_msgs
  grid_map_ros
  interactive_markers
  pluginlib
  Qt5
  rclcpp
  rviz_common
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${Qt5Widgets_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  ${geometry_msgs_TARGETS}
  ${mavros_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${planner_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  rviz_common::rviz_common
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

add_executable(standalone_test
  src/standalone_test.cpp
)

target_link_libraries(standalone_test PUBLIC
  ${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  Qt5
  rviz_common
  geometry_msgs
  rclcpp
)

install(
  TARGETS standalone_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
