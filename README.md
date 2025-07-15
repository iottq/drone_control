# drone_control

Master the integration of ROS2, PX4, and OpenCV to achieve precision landing using ArUco marker detection.You can use your iPhone to control your move via virtual Joystrick.

### Prerequisites
* Raspberry Pi 4B
* Quadrotor PX4 Drone with V1.15 version firmware
* Raspberry camera V2
* USB Serial （CH340）connect to PX4 TELE1

### PX4 parameters setting
* COM_RC_IN_MODE: "RC and Joystick with fackback"
* UXRCE_DDS_CFG: "TELEM1"
* SER_TELE1_BAUD: "921600 8N1"
* MAV_0_CONFIG: "Disabled"

## Flashing images

Find the sdcard image or archive with fastboot images [here](https://github.com/tesla-android/android-raspberry-pi/releases)

Use the SDCard raw image to flash the Android into SDCard.


sudo apt install libgeographic-dev ros-humble-camera-info-manager libboost-all-dev python3-dev
rosdep install --from-paths src/web_video_server -i
rosdep install -y --from-paths src/camera_ros --ignore-src --rosdistro $ROS_DISTRO
pip install bluezero