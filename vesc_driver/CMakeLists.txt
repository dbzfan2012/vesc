cmake_minimum_required(VERSION 3.5)
project(vesc_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vesc_msgs REQUIRED)
find_package(Boost REQUIRED)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

# node executable
add_executable(vesc_driver_node
  src/vesc_driver_node.cpp
  src/vesc_driver.cpp
  src/vesc_interface.cpp
  src/vesc_packet.cpp
  src/vesc_packet_factory.cpp
  src/serial_port.cpp
)
ament_target_dependencies(vesc_driver_node
  rclcpp
  std_msgs
  vesc_msgs
)
target_link_libraries(vesc_driver_node
  ${Boost_LIBRARIES}
)

# Install
install(TARGETS vesc_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(PROGRAMS scripts/throttle_interpolator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
