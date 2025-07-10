#include "geoland_action_server.hpp"

geoland_server::GeolandActionServer::GeolandActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("geoland_action_server", options), context_(*this)
  {
    using namespace std::placeholders;
    
    this->action_server_ = rclcpp_action::create_server<Geoland>(
      this,
      "geoland",
      std::bind(&GeolandActionServer::handle_goal, this, _1, _2),
      std::bind(&GeolandActionServer::handle_cancel, this, _1),
      std::bind(&GeolandActionServer::handle_accepted, this, _1));

    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    // home_position_sub_ = this->create_subscription<HomePosition>(
		// 	"/fmu/out/home_position",qos_profile,std::bind(&geoland_server::GeolandActionServer::home_position_callback, this, _1)
		// );

    local_position_sub_ = this->create_subscription<VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position",qos_profile, std::bind(&geoland_server::GeolandActionServer::local_position_callback, this, _1)
    );

    global_position_sub_ = this->create_subscription<VehicleGlobalPosition>(
      "/fmu/out/vehicle_global_position",qos_profile, std::bind(&geoland_server::GeolandActionServer::global_position_callback, this, _1)
    );

    vehicle_status_sub_ = this->create_subscription<VehicleStatus>(
      "/fmu/out/vehicle_status",qos_profile, std::bind(&geoland_server::GeolandActionServer::vehicle_status_callback, this, _1)
    );
    _target_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("/target_pose",
      rclcpp::QoS(1).best_effort(), std::bind(&geoland_server::GeolandActionServer::targetPoseCallback, this, std::placeholders::_1));

    _vehicle_land_detected_sub = this->create_subscription<px4_msgs::msg::VehicleLandDetected>("/fmu/out/vehicle_land_detected",
        rclcpp::QoS(1).best_effort(), std::bind(&geoland_server::GeolandActionServer::vehicleLandDetectedCallback, this, std::placeholders::_1));

    _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(context_);

    _vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(context_);

    _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(context_);
    loadParameters();
    
  }


  void geoland_server::GeolandActionServer::loadParameters()
  {
    this->declare_parameter<float>("descent_vel", 1.0);
    this->declare_parameter<float>("vel_p_gain", 1.5);
    this->declare_parameter<float>("vel_i_gain", 0.0);
    this->declare_parameter<float>("max_velocity", 3.0);
    this->declare_parameter<float>("target_timeout", 15.0);
    this->declare_parameter<float>("delta_position", 0.25);
    this->declare_parameter<float>("delta_velocity", 0.25);
  
    this->get_parameter("descent_vel", _param_descent_vel);
    this->get_parameter("vel_p_gain", _param_vel_p_gain);
    this->get_parameter("vel_i_gain", _param_vel_i_gain);
    this->get_parameter("max_velocity", _param_max_velocity);
    this->get_parameter("target_timeout", _param_target_timeout);
    this->get_parameter("delta_position", _param_delta_position);
    this->get_parameter("delta_velocity", _param_delta_velocity);
  
    RCLCPP_INFO(this->get_logger(), "descent_vel: %f", _param_descent_vel);
    RCLCPP_INFO(this->get_logger(), "vel_i_gain: %f", _param_vel_i_gain);
  }  
  

rclcpp_action::GoalResponse geoland_server::GeolandActionServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Geoland::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request lat: %6f lon: %6f alt: %6f cmd: %d", goal->latitude, goal->longitude, goal->altitude, goal->cmd);
    std::string uuid_str = rclcpp_action::to_string(uuid);
    RCLCPP_INFO(this->get_logger(), "Received goal UUID: %s", uuid_str.c_str());
    target_latitude_ = goal->latitude, target_longitude_ = goal->longitude , target_altitude_ = goal->altitude;
    cmd = goal->cmd;
    (void)uuid;
    if(!pre_flight_checks_pass_){
      RCLCPP_INFO(this->get_logger(), "pre flight checks failed.");
      return rclcpp_action::GoalResponse::REJECT;
    }
   

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse geoland_server::GeolandActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleGeoland> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void geoland_server::GeolandActionServer::handle_accepted(const std::shared_ptr<GoalHandleGeoland> goal_handle)
  {

     // 如果已有任务在执行，则拒绝新任务
     if (active_goal_handle_ && active_goal_handle_->is_active()) {
      RCLCPP_WARN(this->get_logger(), "A goal is already active. Rejecting new goal.");

      auto result = std::make_shared<Geoland::Result>();
      result->success = false;

      goal_handle->abort(result);  // 立即中止这个新任务
      return;
    }

    active_goal_handle_ = goal_handle;

    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&GeolandActionServer::execute, this, _1), goal_handle}.detach();
  }

  /*  
      1.设置offboard模式
      2.解锁
      3.gpc转ned坐标
      4.发布trajectory_setpoint
      5.订阅px4当前位置，检查是否到达位置
       到达->完成任务
       否则->继续发布setpoint

       cmd: 1 ->  fly to location and return;
            2 ->  fly to location and descend;
            3 ->  descend;
  */

  void geoland_server::GeolandActionServer::execute(const std::shared_ptr<GoalHandleGeoland> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(20);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Geoland::Feedback>();
    auto result = std::make_shared<Geoland::Result>();
    _state = State::Takeoff;
    double distance = 0.0;
    GeographicLib::LocalCartesian geo_converter;

    home_alt_ = current_alt_, home_lat_ = current_lat_, home_lon_ = current_lon_;
    RCLCPP_INFO(this->get_logger(), "Location init: lat: %f lon: %f", home_lat_, home_lon_);
    geo_converter.Reset(home_lat_,home_lon_,home_alt_);
    geo_converter.Forward(target_latitude_,target_longitude_,target_altitude_, target_x_, target_y_, target_z_);
    RCLCPP_INFO(this->get_logger(), "Location taget: lat: %f lon: %f", target_latitude_, target_longitude_);

    _search_started = false;
    _land_detected = false;
    double idle_x = 0, idle_y = 0, idle_z = 0;
    _tag.clear();
    if(cmd==uint32_t(3)){
      switchToState(State::Search);
      geo_converter.Forward(current_lat_,current_lon_,current_z_, target_x_, target_y_, target_z_);
    }
    RCLCPP_INFO(this->get_logger(), "target ned: x=%f y=%f z=%f", float(target_x_), float(target_y_), float(target_z_));
    this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    if(arming_state_ == VehicleStatus::ARMING_STATE_DISARMED){
      this->arm();
    }
    
    do {
      
      publish_offboard_control_mode();
      publish_trajectory_setpoint(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), 0, 0, 0, std::numeric_limits<double>::quiet_NaN());
      

    } while(mode_ != VehicleStatus::NAVIGATION_STATE_OFFBOARD);
    
    RCLCPP_INFO(this->get_logger(), "enter offboard mode.");
    while(rclcpp::ok() && mode_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {

      if(goal_handle->is_canceling()) {

        //position mode.
        this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 3);
        result->success = false;
        result->message = "task terminnal.";
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        goal_handle->canceled(result);
        return;
      }

      bool target_lost = checkTargetTimeout();

      if (target_lost && !_target_lost_prev) {
        RCLCPP_INFO(this->get_logger(), "Target lost: State %s", stateName(_state).c_str());
    
      } else if (!target_lost && _target_lost_prev) {
        RCLCPP_INFO(this->get_logger(), "Target acquired");
      }
      _target_lost_prev = target_lost;

      switch (_state)
      {
          
          case State::Idle: {
              publish_offboard_control_mode();
              publish_trajectory_setpoint(idle_x, idle_y, idle_z, 0, 0, 0 ,0);
              break;
          }
          case State::Search: {
              publish_offboard_control_mode();

              if(!_search_started){
                generateSearchWaypoints();
                _search_started = true;
              }
              
              if (!std::isnan(_tag.position.x())) {
                
                _approach_altitude = _vehicle_local_position->positionNed().z();
                RCLCPP_INFO(this->get_logger(), "approach_alttitude: %f", _approach_altitude);
                switchToState(State::Approach);
                break;
              }
          
              auto waypoint_position = _search_waypoints[_search_waypoint_index];
          
              _trajectory_setpoint->updatePosition(waypoint_position);
          
              if (positionReached(waypoint_position)) {
                _search_waypoint_index++;
          
                // If we have searched all waypoints, start over
                if (_search_waypoint_index >= static_cast<int>(_search_waypoints.size())) {
                  _search_waypoint_index = 0;
                }
              }
              break;
          }

          case State::Approach: {
                publish_offboard_control_mode();
                if (target_lost) {
                  RCLCPP_INFO(this->get_logger(), "Failed! Target lost during %s", stateName(_state).c_str());
                  // idle_x = current_x_;
                  // idle_y = current_y_;
                  // idle_z = current_z_;
                  // switchToState(State::Idle);
                  break;
                }
            
                // Approach using position setpoints
                auto target_position = Eigen::Vector3f(_tag.position.x(), _tag.position.y(), _approach_altitude);
            
                _trajectory_setpoint->updatePosition(target_position);
            
                if (positionReached(target_position)) {
                  switchToState(State::Descend);
                }

                break;
          }

          case State::Descend: {
                publish_offboard_control_mode();
                if (target_lost) {
                  RCLCPP_INFO(this->get_logger(), "Failed! Target lost during %s", stateName(_state).c_str());
                  //switchToState(State::Idle);
                }
            
                // Descend using velocity setpoints and P velocity controller for XY
                Eigen::Vector2f vel = calculateVelocitySetpointXY();
                _trajectory_setpoint->update(Eigen::Vector3f(vel.x(), vel.y(), _param_descent_vel), std::nullopt,
                          px4_ros2::quaternionToYaw(_tag.orientation));

                if(arming_state_ == VehicleStatus::ARMING_STATE_DISARMED && std::abs(current_vz_) < 0.2){
                  _land_detected = true;
                }
                
                //RCLCPP_INFO(this->get_logger(), "Landed: %s", _land_detected ? "true":"false");
                if (_land_detected) {
                  //switchToState(State::Finished);
                    this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 3);
                    result->success = true;
                    result->message = "landed";
                    goal_handle->succeed(result);
                    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                    active_goal_handle_ = nullptr;
                    return;
                }
            
                break;
              
          }

          case State::Finished: {
              break;
          }

          

          default:
              break;
      }

      
      distance = 0;
      feedback->current_distance = distance;
      feedback->current_step = stateName(_state).c_str();
      goal_handle->publish_feedback(feedback);
      loop_rate.sleep();
    }
    // Check if goal is done
    this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 3);
    result->success = true;
    result->message = "Canceled";
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    
  }

   /**
  * @brief Publish the offboard control mode.
  *        For this example, only position and altitude controls are active.
  */
  void geoland_server::GeolandActionServer::publish_offboard_control_mode()
  {
    OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = true;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
  }


 /**
  * @brief Publish vehicle commands
  * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
  * @param param1    Command parameter 1
  * @param param2    Command parameter 2
  */
 void geoland_server::GeolandActionServer::publish_vehicle_command(uint16_t command, float param1, float param2)
 {
	 VehicleCommand msg{};
	 msg.param1 = param1;
	 msg.param2 = param2;
	 msg.command = command;
	 msg.target_system = 1;
	 msg.target_component = 1;
	 msg.source_system = 1;
	 msg.source_component = 1;
	 msg.from_external = true;
	 msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	 vehicle_command_publisher_->publish(msg);
 }


double geoland_server::GeolandActionServer::get_dest_yaw()
{
        // 计算目标位置与当前位置之间的差值
     double delta_x = target_x_ - current_x_;
     double delta_y = target_y_ - current_y_;
     double yaw = std::atan2(delta_x, delta_y); // 结果单位是弧度
  
     return yaw;
}

 /**
  * @brief Send a command to Arm the vehicle
  */
void geoland_server::GeolandActionServer::arm()
{
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

  RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
  * @brief Publish a trajectory setpoint
  *        For this example, it sends a trajectory setpoint to make the
  *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
  */
 void geoland_server::GeolandActionServer::publish_trajectory_setpoint(float x, float y, float z, float vx, float vy, float vz, float yaw)
 {
	 TrajectorySetpoint msg{};
	 msg.position = {x,y,z};
   msg.velocity = {vx, vy, vz};
	 //msg.yaw = -3.14; // [-PI:PI]
   msg.yaw = yaw;
	 msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	 trajectory_setpoint_publisher_->publish(msg);
 }

 void geoland_server::GeolandActionServer::local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
 {
     //RCLCPP_INFO(this->get_logger(), "local z: %f target_z: %f", msg->z, float(z_target_));
    current_x_ = msg->x;
    current_y_ = msg->y;
    current_z_ = msg->z;
    current_vz_ = msg->vz;
    

 }

 void geoland_server::GeolandActionServer::global_position_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
 {
     current_lat_ = msg->lat;
     current_lon_ = msg->lon;
     current_alt_ = msg->alt;

 }

 void geoland_server::GeolandActionServer::vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
 {
    
    arming_state_ = msg->arming_state;
    pre_flight_checks_pass_ = msg->pre_flight_checks_pass;
    mode_ = msg->nav_state;
    //RCLCPP_INFO(this->get_logger(), "pre_flight_checks_pass %s",pre_flight_checks_pass_ ? "true":"false" );

 }

void geoland_server::GeolandActionServer::switchToState(State state)
{
	RCLCPP_INFO(this->get_logger(), "Switching to %s", stateName(state).c_str());
	_state = state;
}

std::string geoland_server::GeolandActionServer::stateName(State state)
{
	switch (state) {
        case State::Takeoff:
          return "Takeoff";

        case State::Flying:
          return "Flying";
        case State::Reach:
          return "Reach";
        case State::Land:
          return "Land";
        case State::Return:
          return "Return";
        case State::Idle:
          return "Idle";
        case State::Search:
          return "Search";
        case State::Approach:
          return "Approach";
        case State::Descend:
          return "Descend";
        case State::Finished:
          return "Finished";
        default:
          return "Unknown";
	}
}

void geoland_server::GeolandActionServer::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	if (_search_started) {
		auto tag = ArucoTag {
			.position = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z),
			.orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z),
			.timestamp = this->now(),
		};

		// Save tag position/orientation in NED world frame
		_tag = getTagWorld(tag);
	}

}

void geoland_server::GeolandActionServer::vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
	_land_detected = msg->landed;
}

geoland_server::GeolandActionServer::ArucoTag geoland_server::GeolandActionServer::getTagWorld(const ArucoTag& tag)
{
	// Convert from optical to NED
	// Optical: X right, Y down, Z away from lens
	// NED: X forward, Y right, Z away from viewer
	Eigen::Matrix3d R;
	R << 0, -1, 0,
	1, 0, 0,
	0, 0, 1;
	Eigen::Quaterniond quat_NED(R);

	auto vehicle_position = Eigen::Vector3d(_vehicle_local_position->positionNed().cast<double>());
	auto vehicle_orientation = Eigen::Quaterniond(_vehicle_attitude->attitude().cast<double>());

	Eigen::Affine3d drone_transform = Eigen::Translation3d(vehicle_position) * vehicle_orientation;
	Eigen::Affine3d camera_transform = Eigen::Translation3d(0, 0, 0) * quat_NED;
	Eigen::Affine3d tag_transform = Eigen::Translation3d(tag.position) * tag.orientation;
	Eigen::Affine3d tag_world_transform = drone_transform * camera_transform * tag_transform;

	ArucoTag world_tag = {
		.position = tag_world_transform.translation(),
		.orientation = Eigen::Quaterniond(tag_world_transform.rotation()),
		.timestamp = tag.timestamp,
	};

	return world_tag;
}

void geoland_server::GeolandActionServer::generateSearchWaypoints()
{
	// Generate spiral search waypoints
	// The search waypoints are generated in the NED frame
	// Parameters for the search pattern
	double start_x = 0.0;
	double start_y = 0.0;
	double current_z = _vehicle_local_position->positionNed().z();
	auto min_z = -1.0;

	double max_radius = 2.0;
	double layer_spacing = 0.5;
	int points_per_layer = 16;
	std::vector<Eigen::Vector3f> waypoints;

	// Generate waypoints
	// Calculate the number of layers needed
	int num_layers = (static_cast<int>((min_z - current_z) / layer_spacing) / 2) < 1 ? 1 : (static_cast<int>((
				 min_z - current_z) / layer_spacing) / 2);

	// Generate waypoints
	for (int layer = 0; layer < num_layers; ++layer) {
		std::vector<Eigen::Vector3f> layer_waypoints;

		// Spiral out to max radius
		double radius = 0.0;

		for (int point = 0; point < points_per_layer + 1; ++point) {
			double angle = 2.0 * M_PI * point / points_per_layer;
			double x = start_x + radius * cos(angle);
			double y = start_y + radius * sin(angle);
			double z = current_z;

			layer_waypoints.push_back(Eigen::Vector3f(x, y, z));
			radius += max_radius / points_per_layer;
		}

		// Push the spiral out waypoints to the main waypoints vector
		waypoints.insert(waypoints.end(), layer_waypoints.begin(), layer_waypoints.end());

		// Decrease the altitude for the inward spiral
		current_z += layer_spacing;

		// Reverse the layer waypoints for spiral in
		std::reverse(layer_waypoints.begin(), layer_waypoints.end());

		// Adjust the z-coordinate for the inward spiral
		for (auto& waypoint : layer_waypoints) {
			waypoint.z() = current_z;
		}

		// Push the reversed waypoints to the main waypoints vector
		waypoints.insert(waypoints.end(), layer_waypoints.begin(), layer_waypoints.end());

		// Decrease the altitude for the next outward spiral
		current_z += layer_spacing;
	}

	_search_waypoints = waypoints;
}

bool geoland_server::GeolandActionServer::checkTargetTimeout()
{
	if (!_tag.valid()) {
		return true;
	}

	if (this->now().seconds() - _tag.timestamp.seconds() > _param_target_timeout) {
		return true;
	}

	return false;
}

bool geoland_server::GeolandActionServer::positionReached(const Eigen::Vector3f& target) const
{
	auto position = _vehicle_local_position->positionNed();
	auto velocity = _vehicle_local_position->velocityNed();

	const auto delta_pos = target - position;
	// NOTE: this does NOT handle a moving target!
	return (delta_pos.norm() < _param_delta_position) && (velocity.norm() < _param_delta_velocity);
}

Eigen::Vector2f geoland_server::GeolandActionServer::calculateVelocitySetpointXY()
{
	float p_gain = _param_vel_p_gain;
	float i_gain = _param_vel_i_gain;

	// P component
	float delta_pos_x = _vehicle_local_position->positionNed().x() - _tag.position.x();
	float delta_pos_y = _vehicle_local_position->positionNed().y() - _tag.position.y();

	// I component
	_vel_x_integral += delta_pos_x;
	_vel_y_integral += delta_pos_y;
	float max_integral = _param_max_velocity;
	_vel_x_integral = std::clamp(_vel_x_integral, -1.f * max_integral, max_integral);
	_vel_y_integral = std::clamp(_vel_y_integral, -1.f * max_integral, max_integral);

	float Xp = delta_pos_x * p_gain;
	float Xi = _vel_x_integral * i_gain;
	float Yp = delta_pos_y * p_gain;
	float Yi = _vel_y_integral * i_gain;

	// Sum P and I gains
	float vx = -1.f * (Xp + Xi);
	float vy = -1.f * (Yp + Yi);

	// 0.1m/s min vel and 3m/s max vel
	vx = std::clamp(vx, -1.f * _param_max_velocity, _param_max_velocity);
	vy = std::clamp(vy, -1.f * _param_max_velocity, _param_max_velocity);

	return Eigen::Vector2f(vx, vy);
}


RCLCPP_COMPONENTS_REGISTER_NODE(geoland_server::GeolandActionServer)