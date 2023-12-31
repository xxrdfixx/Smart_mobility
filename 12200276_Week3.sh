#!/bin/sh


#Set Locale 

locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings


#Enable required repositories
sudo apt install software-properties-common
sudo add-apt-repository universe



sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg


echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null


#Install development tools (optional)
sudo apt update && sudo apt install ros-dev-tools


#Install ROS 2
sudo apt update



sudo apt upgrade


sudo apt install ros-iron-desktop



sudo apt install ros-iron-ros-base


#Setup environment
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setu



#Try some example

#In one terminal, source the setup file and then run a C++ talker:
source /opt/ros/iron/setup.bash
ros2 run demo_nodes_cpp talker


#In another terminal source the setup file and then run a Python listener:
source /opt/ros/iron/setup.bash
ros2 run demo_nodes_py listener
