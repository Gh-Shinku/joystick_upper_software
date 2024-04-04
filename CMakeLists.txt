cmake_minimum_required(VERSION 3.10)

project(joystick_upper_software VERSION 1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(spdlog REQUIRED CONFIG)
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
    joystick_upper_software
    src/main.cpp
)
ament_target_dependencies(joystick_upper_software rclcpp bupt_interfaces)
target_link_libraries(joystick_upper_software communication_lib Boost::system spdlog::spdlog)
target_include_directories(joystick_upper_software PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

if (WIN32)
    add_executable(
        test_winsock
        test/winsock_test.cpp
    )
    target_link_libraries(test_winsock PRIVATE communication_lib)
    target_include_directories(test_winsock PUBLIC ${CMAKE_SOURCE_DIR}/include)
endif()

add_executable(
    test_boost_socket
    test/boost_sock_test.cpp
)
target_link_libraries(test_boost_socket PRIVATE communication_lib Boost::system)
target_include_directories(test_boost_socket PUBLIC ${CMAKE_SOURCE_DIR}/include)

add_executable(
    test_boost_serial
    test/serial_test.cpp
)
target_link_libraries(test_boost_serial PRIVATE communication_lib Boost::system)
target_include_directories(test_boost_serial PUBLIC ${CMAKE_SOURCE_DIR}/include)

install(TARGETS joystick_upper_software
  DESTINATION lib/${PROJECT_NAME})

ament_package()