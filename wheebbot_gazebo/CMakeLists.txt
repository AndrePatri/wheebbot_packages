cmake_minimum_required(VERSION 3.5)
project(wheebbot_gazebo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

find_package(std_msgs REQUIRED)

find_package(ignition-gazebo6 REQUIRED)
find_package(xacro REQUIRED)

find_package(ament_cmake_auto REQUIRED)

ament_auto_package(INSTALL_TO_SHARE launch config worlds)
