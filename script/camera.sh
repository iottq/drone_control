#!/bin/bash

export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
cd /home/ros2/drone_control
source install/setup.bash
ros2 run camera_ros camera_node --ros-args -p format:=RGB888 -p width:=800 -p height:=600 -p FrameDurationLimits:="[100000,100000]"
#ros2 run camera_ros camera_node --ros-args -p format:=RGB888 -p width:=800 -p height:=600
#ros2 run camera_ros camera_node --ros-args -p format:=RGB888 -p width:=1920 -p height:=1080
