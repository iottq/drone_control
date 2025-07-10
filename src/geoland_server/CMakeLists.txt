cmake_minimum_required(VERSION 3.8)
project(geoland_server)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib")
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(action_geoland_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(GeographicLib REQUIRED)

find_package(px4_ros2_cpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

include_directories(${Eigen3_INCLUDE_DIRS})

add_library(action_server SHARED
  src/geoland_action_server.cpp)
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_server
  PRIVATE "GEOLAND_SERVER_BUILDING_DLL")
ament_target_dependencies(action_server
  "action_geoland_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "px4_msgs"
  "GeographicLib"
  "px4_ros2_cpp"
  "Eigen3"
  "geometry_msgs"
  "tf2"
  "tf2_ros")
rclcpp_components_register_node(action_server PLUGIN "geoland_server::GeolandActionServer" EXECUTABLE geoland_action_server)
target_compile_features(action_server PUBLIC c_std_99 cxx_std_17)
install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
