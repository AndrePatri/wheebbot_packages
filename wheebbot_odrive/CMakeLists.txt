cmake_minimum_required(VERSION 3.5)
project(wheebbot_odrive)

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

# Installing directories
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}) # installing diretories so that ros2 can see them

# Installing targets:
install(PROGRAMS
  src/odrv_axes_USBconfigurator.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
