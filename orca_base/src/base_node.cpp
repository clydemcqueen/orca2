#include "orca_base/base_node.hpp"
#include "orca_base/pwm.hpp"

namespace orca_base {

//=============================================================================
// Constants
//=============================================================================

const double Z_HOLD_MAX = -0.05;  // Highest z hold
const double Z_HOLD_MIN = -50;    // Lowest z hold

//=============================================================================
// Thrusters
//=============================================================================

struct Thruster
{
  std::string frame_id;   // URDF link frame id
  bool ccw;               // True if counterclockwise
  double forward_factor;
  double strafe_factor;
  double yaw_factor;
  double vertical_factor;
};

// Order must match the order of the <thruster> tags in the URDF
const std::vector<Thruster> THRUSTERS = {
  {"t200_link_front_right", false, 1.0, 1.0, 1.0, 0.0},
  {"t200_link_front_left", false, 1.0, -1.0, -1.0, 0.0},
  {"t200_link_rear_right", true, 1.0, -1.0, 1.0, 0.0},
  {"t200_link_rear_left", true, 1.0, 1.0, -1.0, 0.0},
  {"t200_link_vertical_right", false, 0.0, 0.0, 0.0, 1.0},
  {"t200_link_vertical_left", true, 0.0, 0.0, 0.0, -1.0},
};

//=============================================================================
// BaseNode
//=============================================================================

BaseNode::BaseNode():
  Node{"base_node"},
  mode_{orca_msgs::msg::Control::DISARMED},
  tilt_{0},
  brightness_{0},
  stability_{1.0}  // Default to stable (might not have an IMU)
{
  // Suppress IDE warnings
  (void) baro_sub_;
  (void) battery_sub_;
  (void) imu_sub_;
  (void) joy_sub_;
  (void) leak_sub_;
  (void) map_sub_;
  (void) odom_sub_;
  (void) spin_timer_;

  // Get parameters
  cxt_.load_parameters(*this);

  // Track changes to parameters
  register_param_change_callback(
    [this](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult
    {
      cxt_.change_parameters(*this, parameters);
      auto result = rcl_interfaces::msg::SetParametersResult();
      result.successful = true;
      return result;
    });

  if (cxt_.use_sim_time_) {
    // The simulated IMU is not rotated
    RCLCPP_INFO(get_logger(), "running in a simulation");
    t_imu_base_ = tf2::Matrix3x3::getIdentity();
  } else {
    // The actual IMU is rotated
    RCLCPP_INFO(get_logger(), "running in real life");
    tf2::Matrix3x3 imu_f_base;
    imu_f_base.setRPY(-M_PI / 2, -M_PI / 2, 0);
    t_imu_base_ = imu_f_base.inverse();
  }

  // ROV PID controllers
  rov_z_pid_ = std::make_shared<pid::Controller>(false, cxt_.rov_z_pid_kp_, cxt_.rov_z_pid_ki_, cxt_.rov_z_pid_kd_);
  rov_yaw_pid_ = std::make_shared<pid::Controller>(true, cxt_.rov_yaw_pid_kp_, cxt_.rov_yaw_pid_ki_,
    cxt_.rov_yaw_pid_kd_);

  // Publications
  control_pub_ = create_publisher<orca_msgs::msg::Control>("control", 1);
  thrust_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("thrust_markers", 1);
  planned_path_pub_ = create_publisher<nav_msgs::msg::Path>("planned_path", 1);
  filtered_path_pub_ = create_publisher<nav_msgs::msg::Path>("filtered_path", 1);
  error_pub_ = create_publisher<orca_msgs::msg::PoseError>("error", 1);

  // Monotonic subscriptions
  baro_sub_ = create_subscription<orca_msgs::msg::Barometer>("barometer",
    [this](const orca_msgs::msg::Barometer::SharedPtr msg) -> void { this->baro_cb_.call(msg); });
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/imu/data",
    [this](const sensor_msgs::msg::Imu::SharedPtr msg) -> void { this->imu_cb_.call(msg); });
  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>("joy",
    [this](const sensor_msgs::msg::Joy::SharedPtr msg) -> void { this->joy_cb_.call(msg); });
  map_sub_ = create_subscription<fiducial_vlam_msgs::msg::Map>("fiducial_map",
    [this](const fiducial_vlam_msgs::msg::Map::SharedPtr msg) -> void { this->map_cb_.call(msg); });
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("filtered_odom",
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) -> void { this->odom_cb_.call(msg); });

  // Other subscriptions
  using std::placeholders::_1;
  auto battery_cb = std::bind(&BaseNode::battery_callback, this, _1);
  battery_sub_ = create_subscription<orca_msgs::msg::Battery>("battery", battery_cb);
  auto leak_cb = std::bind(&BaseNode::leak_callback, this, _1);
  leak_sub_ = create_subscription<orca_msgs::msg::Leak>("leak", leak_cb);

  // Loop will run at ~constant wall speed, switch to ros_timer when it exists
  using namespace std::chrono_literals;
  spin_timer_ = create_wall_timer(50ms, std::bind(&BaseNode::spin_once, this));

  RCLCPP_INFO(get_logger(), "base_node ready");
}

// New barometer reading
void BaseNode::baro_callback(const orca_msgs::msg::Barometer::SharedPtr msg, bool first)
{
  if (first) {
    z_initial_ = -msg->depth;
    z_ = 0;
    RCLCPP_INFO(get_logger(), "barometer adjustment %g", z_initial_);
  } else {
    z_ = -msg->depth - z_initial_;
  }
}

// New battery reading
void BaseNode::battery_callback(const orca_msgs::msg::Battery::SharedPtr msg)
{
  if (msg->low_battery) {
    RCLCPP_ERROR(get_logger(), "low battery (%g volts), disarming", msg->voltage);
    set_mode(orca_msgs::msg::Control::DISARMED);
  }
}

// New IMU reading
void BaseNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // Get yaw
  tf2::Quaternion imu_f_map;
  tf2::fromMsg(msg->orientation, imu_f_map);
  tf2::Matrix3x3 base_f_map = tf2::Matrix3x3(imu_f_map) * t_imu_base_;
  double roll = 0, pitch = 0;
  base_f_map.getRPY(roll, pitch, yaw_);

#if 0
  // NWU to ENU TODO still need this?
  yaw_ += M_PI_2;
#endif

  // Compute a stability metric, used to throttle the pid controllers
  stability_ = std::min(clamp(std::cos(roll), 0.0, 1.0), clamp(std::cos(pitch), 0.0, 1.0));
}

// New input from the gamepad
void BaseNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg, bool first)
{
  if (!first) {
    // Arm/disarm
    if (button_down(msg, joy_msg_, joy_button_disarm_)) {
      RCLCPP_INFO(get_logger(), "disarmed");
      set_mode(orca_msgs::msg::Control::DISARMED);
    } else if (button_down(msg, joy_msg_, joy_button_arm_)) {
      RCLCPP_INFO(get_logger(), "armed, manual");
      set_mode(orca_msgs::msg::Control::MANUAL);
    }

    // If we're disarmed, ignore everything else
    if (mode_ == orca_msgs::msg::Control::DISARMED) {
      joy_msg_ = *msg;
      return;
    }

    // Mode
    if (button_down(msg, joy_msg_, joy_button_manual_)) {
      RCLCPP_INFO(get_logger(), "manual");
      set_mode(orca_msgs::msg::Control::MANUAL);
    } else if (button_down(msg, joy_msg_, joy_button_hold_d_)) {
      if (baro_ok(msg->header.stamp)) {
        set_mode(orca_msgs::msg::Control::HOLD_D);
      } else {
        RCLCPP_ERROR(get_logger(), "barometer not ready, can't hold z");
      }
    } else if (button_down(msg, joy_msg_, joy_button_hold_hd_)) {
      if (imu_ok(msg->header.stamp) && baro_ok(msg->header.stamp)) {
        set_mode(orca_msgs::msg::Control::HOLD_HD);
      } else {
        RCLCPP_ERROR(get_logger(), "barometer and/or IMU not ready, can't hold yaw and z");
      }
    } else if (button_down(msg, joy_msg_, joy_button_keep_station_)) {
      if (odom_ok(msg->header.stamp) && map_cb_.receiving()) {
        set_mode(orca_msgs::msg::Control::KEEP_STATION);
      } else {
        RCLCPP_ERROR(get_logger(), "no odometry and/or no map, can't keep station");
      }
    } else if (button_down(msg, joy_msg_, joy_button_random_)) {
      if (odom_ok(msg->header.stamp) && map_cb_.receiving()) {
        set_mode(orca_msgs::msg::Control::RANDOM_PATH);
      } else {
        RCLCPP_ERROR(get_logger(), "no odometry and/or no map, can't start random path");
      }
    }

    // Yaw trim
    if (holding_yaw() && trim_down(msg, joy_msg_, joy_axis_yaw_trim_)) {
      rov_yaw_pid_->set_target(
        rov_yaw_pid_->target() + msg->axes[joy_axis_yaw_trim_] > 0 ? cxt_.inc_yaw_ : -cxt_.inc_yaw_);
      RCLCPP_INFO(get_logger(), "hold yaw at %g", rov_yaw_pid_->target());
    }

    // Z trim
    if (holding_z() && trim_down(msg, joy_msg_, joy_axis_z_trim_)) {
      rov_z_pid_->set_target(clamp(
        msg->axes[joy_axis_z_trim_] > 0 ? rov_z_pid_->target() + cxt_.inc_z_ : rov_z_pid_->target() - cxt_.inc_z_,
        Z_HOLD_MIN, Z_HOLD_MAX));
      RCLCPP_INFO(get_logger(), "hold z at %g", rov_z_pid_->target());
    }

    // Camera tilt
    if (button_down(msg, joy_msg_, joy_button_tilt_up_)) {
      tilt_ = clamp(tilt_ + cxt_.inc_tilt_, TILT_MIN, TILT_MAX);
      RCLCPP_INFO(get_logger(), "tilt at %d", tilt_);
    } else if (button_down(msg, joy_msg_, joy_button_tilt_down_)) {
      tilt_ = clamp(tilt_ - cxt_.inc_tilt_, TILT_MIN, TILT_MAX);
      RCLCPP_INFO(get_logger(), "tilt at %d", tilt_);
    }

    // Lights
    if (button_down(msg, joy_msg_, joy_button_bright_)) {
      brightness_ = clamp(brightness_ + cxt_.inc_lights_, BRIGHTNESS_MIN, BRIGHTNESS_MAX);
      RCLCPP_INFO(get_logger(), "lights at %d", brightness_);
    } else if (button_down(msg, joy_msg_, joy_button_dim_)) {
      brightness_ = clamp(brightness_ - cxt_.inc_lights_, BRIGHTNESS_MIN, BRIGHTNESS_MAX);
      RCLCPP_INFO(get_logger(), "lights at %d", brightness_);
    }

    // Thrusters
    if (rov_mode()) {
      rov_advance(msg->axes[joy_axis_forward_],
        msg->axes[joy_axis_strafe_],
        msg->axes[joy_axis_yaw_],
        msg->axes[joy_axis_vertical_]);
    }
  }

  joy_msg_ = *msg;
}

// Leak detector
void BaseNode::leak_callback(const orca_msgs::msg::Leak::SharedPtr msg)
{
  if (msg->leak_detected) {
    RCLCPP_ERROR(get_logger(), "leak detected, disarming");
    set_mode(orca_msgs::msg::Control::DISARMED);
  }
}

// New map available
void BaseNode::map_callback(const fiducial_vlam_msgs::msg::Map::SharedPtr msg)
{
  map_ = *msg;
}

// New pose estimate available
void BaseNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg, bool first)
{
  filtered_pose_.from_msg(*msg);
  odom_lag_ = (now() - odom_cb_.curr()).seconds();

  if (!first && auv_mode()) {
    // Publish path for rviz
    if (count_subscribers(filtered_path_pub_->get_topic_name()) > 0) {
      filtered_pose_.add_to_path(filtered_path_);
      filtered_path_pub_->publish(filtered_path_);
    }

    Acceleration u_bar;
    double dt = odom_cb_.dt();
    if (mission_->advance(dt, filtered_pose_, u_bar)) {
      // Acceleration => effort
      efforts_.from_acceleration(u_bar, filtered_pose_.pose.yaw);

      // Throttle back if AUV is unstable
      efforts_.set_forward(efforts_.forward() * stability_);
      efforts_.set_strafe(efforts_.strafe() * stability_);
      efforts_.set_vertical(efforts_.vertical() * stability_);
      efforts_.set_yaw(efforts_.yaw() * stability_);

      publish_control(odom_cb_.curr());

      // Publish error
      if (count_subscribers(error_pub_->get_topic_name()) > 0) {
        orca_msgs::msg::PoseError error_msg;
        error_msg.header.stamp = msg->header.stamp;
        mission_->error().to_msg(error_msg);
        error_pub_->publish(error_msg);
      }
    } else {
      // Stop mission
      set_mode(orca_msgs::msg::Control::DISARMED);
    }
  }
}

void BaseNode::rov_advance(float forward, float strafe, float yaw, float vertical)
{
  double dt = joy_cb_.dt();

  efforts_.set_forward(dead_band(forward, cxt_.input_dead_band_) * cxt_.xy_gain_);
  efforts_.set_strafe(dead_band(strafe, cxt_.input_dead_band_) * cxt_.xy_gain_);

  if (holding_yaw()) {
    efforts_.set_yaw(accel_to_effort_yaw(rov_yaw_pid_->calc(yaw_, dt, 0)) * stability_);
  } else {
    efforts_.set_yaw(dead_band(yaw, cxt_.input_dead_band_) * cxt_.yaw_gain_);
  }

  if (holding_z()) {
    efforts_.set_vertical(accel_to_effort_z(rov_z_pid_->calc(z_, dt, HOVER_ACCEL_Z)) * stability_);
  } else {
    efforts_.set_vertical(dead_band(vertical, cxt_.input_dead_band_) * cxt_.vertical_gain_);
  }

  publish_control(joy_cb_.curr());
}

void BaseNode::all_stop()
{
  brightness_ = 0;
  tilt_ = 0;
  efforts_.all_stop();
  publish_control(now());
}

// TODO always send control message, this will allow DriverNode to abort if BaseNode crashes
void BaseNode::publish_control(const rclcpp::Time &msg_time)
{
  // Combine joystick efforts to get thruster efforts.
  std::vector<double> thruster_efforts = {};
  for (int i = 0; i < THRUSTERS.size(); ++i) {
    // Clamp forward + strafe to xy_gain_
    double xy_effort = clamp(
      efforts_.forward() * THRUSTERS[i].forward_factor + efforts_.strafe() * THRUSTERS[i].strafe_factor,
      -cxt_.xy_gain_, cxt_.xy_gain_);

    // Clamp total thrust
    thruster_efforts.push_back(
      clamp(xy_effort + efforts_.yaw() * THRUSTERS[i].yaw_factor + efforts_.vertical() * THRUSTERS[i].vertical_factor,
        THRUST_FULL_REV, THRUST_FULL_FWD));
  }

  // Publish control message
  orca_msgs::msg::Control control_msg;
  control_msg.header.stamp = msg_time;
  control_msg.mode = mode_;
  control_msg.camera_tilt_pwm = tilt_to_pwm(tilt_);
  control_msg.brightness_pwm = brightness_to_pwm(brightness_);
  for (int i = 0; i < thruster_efforts.size(); ++i) {
    control_msg.thruster_pwm.push_back(effort_to_pwm(thruster_efforts[i]));
  }
  control_msg.stability = stability_;
  control_msg.odom_lag = odom_lag_;
  control_pub_->publish(control_msg);

  // Publish rviz thrust markers
  if (count_subscribers(thrust_marker_pub_->get_topic_name()) > 0) {
    visualization_msgs::msg::MarkerArray markers_msg;
    for (int i = 0; i < thruster_efforts.size(); ++i) {
      int32_t action =
        thruster_efforts[i] == 0.0 ? visualization_msgs::msg::Marker::DELETE : visualization_msgs::msg::Marker::ADD;
      double scale = (THRUSTERS[i].ccw ? -thruster_efforts[i] : thruster_efforts[i]) / 5.0;
      double offset = scale > 0 ? -0.1 : 0.1;

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = THRUSTERS[i].frame_id;
      marker.header.stamp = msg_time;
      marker.ns = "thruster";
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = action;
      marker.pose.position.x = 0;
      marker.pose.position.y = 0;
      marker.pose.position.z = offset;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.7071068;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 0.7071068;
      marker.scale.x = scale;
      marker.scale.y = 0.01;
      marker.scale.z = 0.01;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      markers_msg.markers.push_back(marker);
    }
    thrust_marker_pub_->publish(markers_msg);
  }
}

// Change operation mode
void BaseNode::set_mode(uint8_t new_mode)
{
  // Stop all thrusters when we change modes
  efforts_.all_stop();

  if (is_z_hold_mode(new_mode)) {
    rov_z_pid_->set_target(z_);
    RCLCPP_INFO(get_logger(), "hold z at %g", rov_z_pid_->target());
  }

  if (is_yaw_hold_mode(new_mode)) {
    rov_yaw_pid_->set_target(yaw_);
    RCLCPP_INFO(get_logger(), "hold yaw at %g", rov_yaw_pid_->target());
  }

  if (new_mode == orca_msgs::msg::Control::DISARMED) {
    all_stop();
  }

  if (is_auv_mode(new_mode)) {
    std::shared_ptr<BasePlanner> planner;
    if (new_mode == orca_msgs::msg::Control::KEEP_STATION) {
      planner = std::make_shared<KeepStationPlanner>();
    } else {
      planner = std::make_shared<DownRandomPlanner>();
    }
    mission_ = std::make_shared<Mission>(get_logger(), planner, cxt_, map_, filtered_pose_);

    // Publish path for rviz
    if (count_subscribers(planned_path_pub_->get_topic_name()) > 0) {
      planned_path_pub_->publish(mission_->planned_path());
    }

    // Init filtered_path
    filtered_path_.header.stamp = joy_cb_.curr();
    filtered_path_.header.frame_id = cxt_.map_frame_;
    filtered_path_.poses.clear();
  }

  // Set the new mode
  mode_ = new_mode;
}

void BaseNode::spin_once()
{
  // Ignore 0
  auto spin_time = now();
  if (spin_time.nanoseconds() <= 0) {
    return;
  }

  if (rov_mode() && !joy_ok(spin_time)) {
    RCLCPP_ERROR(get_logger(), "lost joystick during ROV operation, disarming");
    set_mode(orca_msgs::msg::Control::DISARMED);
  }

  if (auv_mode() && !odom_ok(spin_time)) {
    RCLCPP_ERROR(get_logger(), "lost odometry during AUV operation, disarming");
    set_mode(orca_msgs::msg::Control::DISARMED);
  }

  if (auv_mode() && stability_ < 0.4) {
    RCLCPP_ERROR(get_logger(), "excessive tilt during AUV operation, disarming");
    set_mode(orca_msgs::msg::Control::DISARMED);
  }

  if (holding_z() && !baro_ok(spin_time)) {
    RCLCPP_ERROR(get_logger(), "lost barometer while holding z, disarming");
    set_mode(orca_msgs::msg::Control::DISARMED);
  }

  if (holding_yaw() && !imu_ok(spin_time)) {
    RCLCPP_ERROR(get_logger(), "lost IMU while holding yaw, disarming");
    set_mode(orca_msgs::msg::Control::DISARMED);
  }

  if (!auv_mode() && is_auv_mode(cxt_.auto_start_) && !joy_ok(spin_time) && odom_ok(spin_time)) {
    RCLCPP_INFO(get_logger(), "auto-starting mission %d", cxt_.auto_start_);
    set_mode(cxt_.auto_start_);
  }
}

} // namespace orca_base

//=============================================================================
// Main
//=============================================================================

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Init node
  auto node = std::make_shared<orca_base::BaseNode>();

  // Spin node
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
