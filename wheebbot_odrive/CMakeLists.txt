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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Installing directories
install(DIRECTORY launch config src DESTINATION share/${PROJECT_NAME}) # installing diretories so that ros2 can see them

# Include directories:
#include_directories(include)

# Installing targets:
install(PROGRAMS
  src/odrv_axes_USBconfigurator.py
  DESTINATION lib/${PROJECT_NAME}
)

# install(TARGETS
#   talker
#   DESTINATION lib/${PROJECT_NAME})

ament_package()
