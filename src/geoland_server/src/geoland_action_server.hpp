#pragma once
#include <functional>
#include <memory>
#include <thread>

#include "action_geoland_interfaces/action/geo_land.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/home_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <GeographicLib/LocalCartesian.hpp>
#include <cmath>
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>


using namespace px4_msgs::msg;
namespace geoland_server
{
class GeolandActionServer : public rclcpp::Node
{
public:
  using Geoland = action_geoland_interfaces::action::GeoLand;
  using GoalHandleGeoland = rclcpp_action::ServerGoalHandle<Geoland>;

  
  GeolandActionServer(const rclcpp::NodeOptions &option);

private:
  rclcpp_action::Server<Geoland>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const Geoland::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleGeoland> goal_handle);

  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<HomePosition>::SharedPtr home_position_sub_;
  rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr global_position_sub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _target_pose_sub;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr _vehicle_land_detected_sub;

  std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
	std::shared_ptr<px4_ros2::OdometryAttitude> _vehicle_attitude;
  std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
  enum class State {
		Takeoff,
    Flying,
    Land,
    Reach,
    Return,
    Idle,
    Search,
    Approach,
    Descend,
    Finished
	};
  void switchToState(State state);
	std::string stateName(State state);

  const double PI = 3.14159265358979323846;
  const double R = 6371.0;  // 地球半径，单位：公里   

  double home_lat_ = 0.0;
  double home_lon_ = 0.0;
  double home_alt_ = 0.0;
  double current_lat_ = 0.0, current_lon_ = 0.0, current_alt_ = 0.0;
  double target_latitude_ = 0.0 , target_longitude_ = 0.0 , target_altitude_ = 0.0;
  double target_x_ = 0.0, target_y_ = 0.0, target_z_ = 0.0;
  float current_x_ = 0.0f, current_y_ = 0.0f, current_z_ = 0.0f, current_vz_ = 0.0f;
  State _state = State::Takeoff;
  uint8_t arming_state_ = VehicleStatus::ARMING_STATE_DISARMED;
  int32_t cmd = 1;
  bool pre_flight_checks_pass_ = false;
  uint8_t mode_ = 0;


  struct ArucoTag {
		// Initialize position with NaN values directly in the struct
		Eigen::Vector3d position = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
		Eigen::Quaterniond orientation;
		rclcpp::Time timestamp;

		bool valid() { return timestamp.nanoseconds() > 0; };

    void clear() {
      position = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
      orientation = Eigen::Quaterniond::Identity();
      timestamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
	};

  bool _search_started = false;

	ArucoTag _tag;
	float _approach_altitude = {};

	// Land detection
	bool _land_detected = false;
	bool _target_lost_prev = true;

	// Waypoints for Search pattern
	std::vector<Eigen::Vector3f> _search_waypoints;
	// Search pattern generation
	void generateSearchWaypoints();
	// Search pattern index
	int _search_waypoint_index = 0;

	// Parameters
	float _param_descent_vel = {};
	float _param_vel_p_gain = {};
	float _param_vel_i_gain = {};
	float _param_max_velocity = {};
	float _param_target_timeout = {};
	float _param_delta_position = {};
	float _param_delta_velocity = {};

	float _vel_x_integral {};
	float _vel_y_integral {};
  px4_ros2::Context context_;
  std::shared_ptr<geoland_server::GeolandActionServer::GoalHandleGeoland> active_goal_handle_ = nullptr;
  

  void handle_accepted(const std::shared_ptr<GoalHandleGeoland> goal_handle);

  void execute(const std::shared_ptr<GoalHandleGeoland> goal_handle);
  void home_position_callback(const px4_msgs::msg::HomePosition::SharedPtr msg);
  void location_to_target_ned(GeographicLib::LocalCartesian &converter,double lat, double lon, double alt);
  void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
  void arm();
  void publish_offboard_control_mode();
  void publish_trajectory_setpoint(float x, float y, float z, float vx, float vy, float vz, float yaw);
  void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void global_position_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
  double get_dest_yaw();
  void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  

  void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
	void vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);
  ArucoTag getTagWorld(const ArucoTag& tag);
  void loadParameters();
  bool checkTargetTimeout();
  bool positionReached(const Eigen::Vector3f& target) const;
  Eigen::Vector2f calculateVelocitySetpointXY();

};  // class GeolandActionServer

}  // namespace action_tutorials_cpp

