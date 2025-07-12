#!/bin/bash

source /opt/ros/humble/setup.bash
cd /home/ros2/drone_control
source install/setup.bash
ros2 run aruco_tracker aruco_tracker
#ros2 run aruco_tracker aruco_tracker --ros-args -p image_topic:=/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image -p camera_info_topic:=/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info
