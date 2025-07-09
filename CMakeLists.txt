cmake_minimum_required(VERSION 3.10)

project(joystick_upper_software VERSION 1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(spdlog REQUIRED CONFIG)
find_package(fmt REQUIRED CONFIG)
find_package(Boost REQUIRED COMPONENTS system)
find_package(rclcpp REQUIRED)
find_package(bupt_interfaces REQUIRED)
find_package(tf2 REQUIRED)

add_definitions(-DROS2)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)

add_library(
  communication_lib STATIC
  src/message.cpp
  src/serial_port.cpp
)
ament_target_dependencies(communication_lib PUBLIC rclcpp)
target_link_libraries(communication_lib PUBLIC spdlog::spdlog Boost::system fmt::fmt)
target_include_directories(communication_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# 手柄报文发布节点
add_executable(
  joystick_publisher
  src/joystick_publisher.cpp
)
ament_target_dependencies(joystick_publisher PUBLIC rclcpp bupt_interfaces)
target_link_libraries(joystick_publisher PRIVATE communication_lib Boost::system spdlog::spdlog fmt::fmt)
target_include_directories(joystick_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# 手柄摇杆处理节点
add_executable(
  joystick_handle_controller
  src/joystick_handle_controller.cpp
)
ament_target_dependencies(joystick_handle_controller PUBLIC rclcpp bupt_interfaces nav_msgs geometry_msgs)
target_link_libraries(joystick_handle_controller PRIVATE communication_lib Boost::system spdlog::spdlog)
target_include_directories(joystick_handle_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# 手柄按键处理节点
add_executable(
  joystick_button_controller
  src/joystick_button_controller.cpp
)
ament_target_dependencies(joystick_button_controller PUBLIC rclcpp std_msgs geometry_msgs nav_msgs bupt_interfaces tf2 tf2_geometry_msgs)
target_include_directories(joystick_button_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(
  TARGETS
  joystick_publisher
  joystick_handle_controller
  joystick_button_controller
  DESTINATION
  lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  launch
  DESTINATION
  share/${PROJECT_NAME}
)

ament_package()
