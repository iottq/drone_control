#!/bin/bash

source /opt/ros/humble/setup.bash
cd /home/zhong/ros2_flycargo
source install/setup.bash
#ros2 run web_video_server web_video_server --ros-args -p default_stream_type:=vp8 -p server_threads:=4 -p verbose:=true
ros2 run web_video_server web_video_server --ros-args -p server_threads:=10 -p verbose:=true
#http://192.168.60.42:8080/stream?topic=/camera/image_raw&type=h264&bitrate=500000
