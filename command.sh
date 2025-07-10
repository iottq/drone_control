#ros2 action send_goal /geoland action_geoland_interfaces/action/GeoLand "{latitude: 47.397971, longitude: 8.546159, altitude: 5, cmd: 2}" --feedback
ros2 action send_goal /geoland action_geoland_interfaces/action/GeoLand "{latitude: 47.397574, longitude: 8.547727, altitude: 5, cmd: 3}" --feedback
#ros2 run geoland_server geoland_action_server
#colcon build --packages-select geoland_server --cmake-clean-cache
#colcon build packages-select action_geoland_interfaces
#journalctl -u aruco_tracker.service -f
#ros2 topic echo /fmu/in/trajectory_setpoint
#sudo systemctl restart geoland_server
ros2 run rqt_image_view rqt_image_view /camera/image_raw
