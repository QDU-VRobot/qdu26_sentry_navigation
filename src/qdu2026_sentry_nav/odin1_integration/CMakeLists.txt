cmake_minimum_required(VERSION 3.8)
project(odin1_integration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# TF Inverter Node
add_executable(tf_inverter_node
  src/tf_inverter_node.cpp
)
ament_target_dependencies(tf_inverter_node
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)

# Point Cloud Fusion Node
add_executable(pointcloud_fusion_node
  src/pointcloud_fusion_node.cpp
)
ament_target_dependencies(pointcloud_fusion_node
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  tf2
  tf2_ros
  tf2_eigen
)
target_link_libraries(pointcloud_fusion_node
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

# Odometry to TF Node
add_executable(odometry_to_tf_node
  src/odometry_to_tf_node.cpp
)
ament_target_dependencies(odometry_to_tf_node
  rclcpp
  nav_msgs
  tf2_ros
  geometry_msgs
)

# Goal Status Node
add_executable(goal_status_node
  src/goal_status_node.cpp
)
ament_target_dependencies(goal_status_node
  rclcpp
  std_msgs
  action_msgs
)

# Approach Velocity Controller
add_executable(approach_velocity_controller
  src/approach_velocity_controller.cpp
)
ament_target_dependencies(approach_velocity_controller
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)

install(TARGETS
  tf_inverter_node
  pointcloud_fusion_node
  odometry_to_tf_node
  goal_status_node
  approach_velocity_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
