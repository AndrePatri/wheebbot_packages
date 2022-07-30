#!/bin/bash
source /opt/ros/foxy/setup.bash
source ~/WheeBBot/WheeBBot_v1/ROS2_foxy/wheebbot_ws/install/local_setup.sh # sourcing foxy workspace

ros2 run xacro xacro wheebbot.urdf.xacro > wheebbot.urdf