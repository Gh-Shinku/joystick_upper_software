cmake_minimum_required(VERSION 3.10)

project(joystick_upper_software VERSION 1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
#find_package(spdlog REQUIRED CONFIG)
find_package(Boost REQUIRED COMPONENTS system)
find_package(rclcpp REQUIRED)
find_package(bupt_interfaces REQUIRED)

add_definitions(-DROS2_ENVIROMENT)

set(CMAKE_CXX_STANDARD 17)

add_library(
    communication_lib STATIC
    src/message.cpp
    src/serial_port.cpp
    src/tcp_socket.cpp
)
target_link_libraries(communication_lib PRIVATE spdlog::spdlog Boost::system)
target_include_directories(communication_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(
    joystick_publisher
    src/joystick_publisher.cpp
)
ament_target_dependencies(joystick_publisher rclcpp bupt_interfaces)
target_link_libraries(joystick_publisher communication_lib Boost::system spdlog::spdlog)
target_include_directories(joystick_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(
    joystick_controller
    src/joystick_controller.cpp
)
ament_target_dependencies(joystick_controller rclcpp bupt_interfaces geometry_msgs)
target_link_libraries(joystick_controller communication_lib Boost::system spdlog::spdlog)
target_include_directories(joystick_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS joystick_publisher joystick_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()