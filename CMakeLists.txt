cmake_minimum_required(VERSION 3.8)
project(bme_gazebo_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteTargetPose.action"
  DEPENDENCIES geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(motion_executor_component SHARED
  src/motion_executor_component.cpp
)

target_include_directories(motion_executor_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_definitions(motion_executor_component
  PRIVATE BME_GAZEBO_SENSORS_BUILDING_LIBRARY
)

ament_target_dependencies(motion_executor_component
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

target_link_libraries(motion_executor_component
  ${cpp_typesupport_target}
)

rclcpp_components_register_nodes(motion_executor_component
  "bme_gazebo_sensors::MotionExecutorComponent"
)

add_library(goal_bridge_component SHARED
  src/goal_bridge_component.cpp
)

target_include_directories(goal_bridge_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_definitions(goal_bridge_component
  PRIVATE BME_GAZEBO_SENSORS_BUILDING_LIBRARY
)

ament_target_dependencies(goal_bridge_component
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

target_link_libraries(goal_bridge_component
  ${cpp_typesupport_target}
)

rclcpp_components_register_nodes(goal_bridge_component
  "bme_gazebo_sensors::GoalBridgeComponent"
)

add_executable(pose_input_node
  src/pose_input_node.cpp
)

ament_target_dependencies(pose_input_node
  rclcpp
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

install(
  TARGETS
    motion_executor_component
    goal_bridge_component
    pose_input_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY
    launch
    worlds
    rviz
    urdf
    meshes
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)

ament_package()