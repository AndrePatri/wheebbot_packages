cmake_minimum_required(VERSION 3.5)
project(wheebbot_horizon)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Required packages:

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

find_package(std_msgs REQUIRED)

find_package(ignition-gazebo6 REQUIRED)
find_package(xacro REQUIRED)

# Installing directories
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}) # installing diretories so that ros2 can see them

install(PROGRAMS
  src/test_balancing_mpc.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
