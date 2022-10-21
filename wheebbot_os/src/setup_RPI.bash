#!/bin/bash 
sudo apt update

sudo apt upgrade 

sudo apt install can-utils

echo "dtoverlay=seeed-can-fd-hat-v2" | sudo tee -a /boot/firmware/config.txt # needs a reboot to be effective

sudo touch /etc/systemd/network/80-can.network

#echo
#'[Match]
#Name=can*
#
#[CAN]
#BitRate=1000000' >> sudo tee -a /etc/systemd/network/80-can.network


locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

apt-cache policy | grep universe

sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade
sudo apt install ros-humble-ros-base
