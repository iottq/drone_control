#!/bin/bash

export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
cd /home/ros2/drone_control
source install/setup.bash
ros2 run geoland_server geoland_action_server
