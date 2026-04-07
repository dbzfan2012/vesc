cmake_minimum_required(VERSION 3.5)
project(vesc_ackermann)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(vesc_msgs REQUIRED)

include_directories(include)

# ackermann_to_vesc node
add_executable(ackermann_to_vesc_node
  src/ackermann_to_vesc_node.cpp
  src/ackermann_to_vesc.cpp
)
ament_target_dependencies(ackermann_to_vesc_node
  rclcpp std_msgs ackermann_msgs vesc_msgs
)

# vesc_to_odom node
add_executable(vesc_to_odom_node
  src/vesc_to_odom_node.cpp
  src/vesc_to_odom.cpp
)
ament_target_dependencies(vesc_to_odom_node
  rclcpp nav_msgs std_msgs geometry_msgs tf2_ros vesc_msgs
)

# Install
install(TARGETS ackermann_to_vesc_node vesc_to_odom_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
