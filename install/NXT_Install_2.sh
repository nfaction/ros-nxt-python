#!/bin/bash
echo "Installing NXT-Mindstorm ROS for Ubuntu 11.04"

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu natty main" > /etc/apt/sources.list.d/ros-latest.list'

wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

sudo apt-get update
sudo apt-get install ros-electric-nxtall


