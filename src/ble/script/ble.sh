#!/bin/bash

export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
cd /home/zhong/ros2_flycargo
source install/setup.bash
ros2 run ble ble_node
