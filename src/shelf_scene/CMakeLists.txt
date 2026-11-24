cmake_minimum_required(VERSION 3.10)
project(shelf_scene)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(shelf_planning REQUIRED)
find_package(tinyxml2 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(shelf_geometry STATIC src/layout.cpp src/world.cpp)
target_include_directories(shelf_geometry PUBLIC include)

add_library(shelf_process STATIC src/process.cpp)
target_include_directories(shelf_process PUBLIC include)

add_executable(gazebo_world src/gazebo_world.cpp)
target_link_libraries(gazebo_world shelf_geometry)

add_executable(publish_scene src/publish_scene.cpp)
ament_target_dependencies(publish_scene geometry_msgs moveit_msgs rclcpp shape_msgs visualization_msgs)
target_link_libraries(publish_scene shelf_geometry)

add_executable(fixed_grasp src/fixed_grasp.cpp)
ament_target_dependencies(fixed_grasp control_msgs gazebo_msgs geometry_msgs moveit_msgs
  rclcpp rclcpp_action sensor_msgs shape_msgs shelf_planning)
target_link_libraries(fixed_grasp shelf_geometry nlohmann_json::nlohmann_json)

add_executable(baseline src/baseline.cpp src/panda.cpp)
ament_target_dependencies(baseline ament_index_cpp gazebo_msgs rcl_interfaces rclcpp)
target_link_libraries(baseline shelf_geometry shelf_process tinyxml2::tinyxml2 yaml-cpp)

add_executable(compare src/compare.cpp)
target_link_libraries(compare shelf_process nlohmann_json::nlohmann_json)

install(TARGETS baseline compare fixed_grasp gazebo_world publish_scene
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
