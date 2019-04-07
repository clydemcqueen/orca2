#include "orca_base/orca_base.hpp"
#include "orca_base/orca_pwm.hpp"

namespace orca_base {

//=============================================================================
// Constants
//=============================================================================

const double Z_HOLD_MAX = -0.05;  // Highest z hold
const double Z_HOLD_MIN = -50;    // Lowest z hold
const int SPIN_RATE = 10;         // Message publish rate in Hz

const rclcpp::Duration COMM_TIMEOUT_DISARM{5000000000};   // Disarm if we can't communicate with the topside

//=============================================================================
// Utilities
//=============================================================================

bool button_down(const sensor_msgs::msg::Joy::SharedPtr &curr, const sensor_msgs::msg::Joy &prev, int button)
{
  return curr->buttons[button] && !prev.buttons[button];
}

bool trim_down(const sensor_msgs::msg::Joy::SharedPtr &curr, const sensor_msgs::msg::Joy &prev, int axis)
{
  return curr->axes[axis] && !prev.axes[axis];
}

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
// OrcaBase node
//=============================================================================

OrcaBase::OrcaBase():
  Node{"orca_base"},
  simulation_{true},
  mode_{orca_msgs::msg::Control::DISARMED},
  imu_ready_{false},
  barometer_ready_{false},
  tilt_{0},
  brightness_{0},
  prev_loop_time_{now()},
  prev_joy_time_{now()},
  z_controller_{false, 0.1, 0, 0.05}, // TODO params
  yaw_controller_{true, 0.007, 0, 0}      // TODO params
{
  // Suppress IDE warnings
  (void)baro_sub_;
  (void)battery_sub_;
  (void)goal_sub_;
  (void)gps_sub_;
  (void)ground_truth_sub_;
  (void)imu_sub_;
  (void)joy_sub_;
  (void)leak_sub_;
  (void)odom_local_sub_;

  // TODO parameters
  inc_yaw_ = M_PI/36;
  inc_z_ = 0.1;
  inc_tilt_ = 5;
  inc_lights_ = 20;
  input_dead_band_ = 0.05f;     // Don't respond to tiny joystick movements
  yaw_pid_dead_band_ = 0.0005;
  z_pid_dead_band_ = 0.002;
  xy_gain_ = 0.5;
  yaw_gain_ = 0.2;
  vertical_gain_ = 0.5;

  if (simulation_) {
    // The simulated IMU is not rotated
    RCLCPP_INFO(get_logger(), "running in a simulation");
    imu_rotation_ = tf2::Matrix3x3::getIdentity();
  } else {
    // The actual IMU is rotated
    RCLCPP_INFO(get_logger(), "running in real life");
    tf2::Matrix3x3 imu_orientation;
    imu_orientation.setRPY(-M_PI/2, -M_PI/2, 0);
    imu_rotation_ =  imu_orientation.inverse();
  }

  // Publications
  control_pub_ = create_publisher<orca_msgs::msg::Control>("/orca_base/control", 1);
  odom_plan_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odometry/plan", 1);
  thrust_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("thrust_markers", 1);
  mission_plan_pub_ = create_publisher<nav_msgs::msg::Path>("mission_plan", 1);
  mission_estimated_pub_ = create_publisher<nav_msgs::msg::Path>("mission_estimated", 1);
  mission_ground_truth_pub_ = create_publisher<nav_msgs::msg::Path>("mission_ground_truth", 1);

  // Callbacks
  using std::placeholders::_1;
  auto baro_cb = std::bind(&OrcaBase::baro_callback, this, _1);
  auto battery_cb = std::bind(&OrcaBase::battery_callback, this, _1);
  auto goal_cb = std::bind(&OrcaBase::goal_callback, this, _1);
  auto gps_cb = std::bind(&OrcaBase::gps_callback, this, _1);
  auto ground_truth_cb = std::bind(&OrcaBase::ground_truth_callback, this, _1);
  auto imu_cb = std::bind(&OrcaBase::imu_callback, this, _1);
  auto joy_cb = std::bind(&OrcaBase::joy_callback, this, _1);
  auto leak_cb = std::bind(&OrcaBase::leak_callback, this, _1);
  auto odom_local_cb = std::bind(&OrcaBase::odom_local_callback, this, _1);

  // Subscriptions
  baro_sub_ = create_subscription<orca_msgs::msg::Barometer>("/barometer", baro_cb);
  battery_sub_ = create_subscription<orca_msgs::msg::Battery>("/orca_driver/battery", battery_cb);
  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", goal_cb);
  gps_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/gps", gps_cb);
  ground_truth_sub_ = create_subscription<nav_msgs::msg::Odometry>("/ground_truth", ground_truth_cb);
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/imu/data", imu_cb);
  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>("/joy", joy_cb);
  leak_sub_ = create_subscription<orca_msgs::msg::Leak>("/orca_driver/leak", leak_cb);
  odom_local_sub_ = create_subscription<nav_msgs::msg::Odometry>("/ground_truth", odom_local_cb); // TODO
  //odom_local_sub_ = create_subscription<nav_msgs::msg::Odometry>("/camera1/filtered_odom", odom_local_cb); // TODO

  RCLCPP_INFO(get_logger(), "orca_base ready");
}

// New barometer reading
void OrcaBase::baro_callback(const orca_msgs::msg::Barometer::SharedPtr msg)
{
  if (!barometer_ready_) {
    // First barometer reading: calibrate
    z_adjustment_ = -msg->depth;
    z_state_ = 0;
    barometer_ready_ = true;
    RCLCPP_INFO(get_logger(), "barometer ready, z adjustment %g", z_adjustment_);
  } else {
    z_state_ = -msg->depth - z_adjustment_;
  }
}

// New battery reading
void OrcaBase::battery_callback(const orca_msgs::msg::Battery::SharedPtr msg)
{
  if (msg->low_battery) {
    RCLCPP_ERROR(get_logger(), "low battery (%g volts), disarming", msg->voltage);
    set_mode(orca_msgs::msg::Control::DISARMED);
  }
}

// New 2D goal
void OrcaBase::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // TODO move some of this logic to set_mode
  if (mode_ != orca_msgs::msg::Control::DISARMED && barometer_ready_ && imu_ready_) {
    // Start plan at current estimate
    odom_plan_.pose = odom_local_;
    odom_plan_.stop_motion();

    // Pull out yaw (kinda cumbersome)
    tf2::Quaternion goal_orientation;
    tf2::fromMsg(msg->pose.orientation, goal_orientation);
    double roll = 0, pitch = 0, goal_yaw = 0;
    tf2::Matrix3x3(goal_orientation).getRPY(roll, pitch, goal_yaw);

    OrcaPose goal_pose(msg->pose.position.x, msg->pose.position.y, odom_plan_.pose.z, goal_yaw);

    RCLCPP_INFO(get_logger(), "start mission at (%g, %g, %g), goal is (%g, %g, %g), yaw %g",
      odom_plan_.pose.x, odom_plan_.pose.y, odom_plan_.pose.z, goal_pose.x, goal_pose.y, goal_pose.z, goal_pose.yaw);

    mission_.reset(new SquareMission(get_logger()));
    if (mission_->init(now(), goal_pose, odom_plan_)) {
      mission_plan_path_.header.stamp = now();
      mission_plan_path_.header.frame_id = "odom";
      mission_plan_path_.poses.clear();

      mission_estimated_path_.header.stamp = now();
      mission_estimated_path_.header.frame_id = "odom";
      mission_estimated_path_.poses.clear();

      mission_ground_truth_path_.header.stamp = now();
      mission_ground_truth_path_.header.frame_id = "odom";
      mission_ground_truth_path_.poses.clear();

      set_mode(orca_msgs::msg::Control::MISSION);
    } else {
      RCLCPP_ERROR(get_logger(), "can't initialize mission, internal error");
    }
  } else {
    RCLCPP_ERROR(get_logger(), "can't start mission, possible reasons: disarmed, barometer not ready, IMU not ready");
  }
}

// New GPS reading
void OrcaBase::gps_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  if (!gps_ready_) {
    gps_ready_ = true;
    RCLCPP_INFO(get_logger(), "GPS ready (%g, %g, %g)",
      msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  }
}

// New ground truth message
void OrcaBase::ground_truth_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!ground_truth_ready_) {
    ground_truth_ready_ = true;
    RCLCPP_INFO(get_logger(), "ground truth available");
  }

  odom_ground_truth_.from_msg(*msg);
}

// New IMU reading
void OrcaBase::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // The IMU was rotated before mounting, so the orientation reading needs to be rotated back.
  tf2::Quaternion imu_orientation;
  tf2::fromMsg(msg->orientation, imu_orientation);
  base_orientation_ = tf2::Matrix3x3(imu_orientation) * imu_rotation_;

  // Get Euler angles
  double roll = 0, pitch = 0;
  base_orientation_.getRPY(roll, pitch, yaw_state_);

  // Compute a stability metric, used to throttle the pid controllers
  stability_ = std::min(clamp(std::cos(roll), 0.0, 1.0), clamp(std::cos(pitch), 0.0, 1.0));

#if 0
  // NWU to ENU TODO do we need this?
  yaw_state_ += M_PI_2;
  base_orientation_.setRPY(roll, pitch, yaw_state_);
#endif

  if (!imu_ready_) {
    imu_ready_ = true;
    RCLCPP_INFO(get_logger(), "IMU ready, roll %4.2f pitch %4.2f yaw %4.2f", roll, pitch, yaw_state_);
  }
}

// Leak detector
void OrcaBase::leak_callback(const orca_msgs::msg::Leak::SharedPtr msg)
{
  if (msg->leak_detected) {
    RCLCPP_ERROR(get_logger(), "leak detected, disarming");
    set_mode(orca_msgs::msg::Control::DISARMED);
  }
}

// Local odometry -- result from robot_localization, fusing all continuous sensors
void OrcaBase::odom_local_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odom_local_.from_msg(*msg);
}

void OrcaBase::publish_odom()
{
  nav_msgs::msg::Odometry odom_msg;

  odom_msg.header.stamp = now(); // TODO motion time
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";

  odom_plan_.to_msg(odom_msg);

  odom_plan_pub_->publish(odom_msg);
}

void OrcaBase::publish_control()
{
  // Combine joystick efforts to get thruster efforts.
  std::vector<double> thruster_efforts = {};
  for (int i = 0; i < THRUSTERS.size(); ++i) {
    // Clamp forward + strafe to xy_gain_
    double xy_effort = clamp(efforts_.forward * THRUSTERS[i].forward_factor + efforts_.strafe * THRUSTERS[i].strafe_factor,
      -xy_gain_, xy_gain_);

    // Clamp total thrust
    thruster_efforts.push_back(clamp(xy_effort + efforts_.yaw * THRUSTERS[i].yaw_factor + efforts_.vertical * THRUSTERS[i].vertical_factor,
      THRUST_FULL_REV, THRUST_FULL_FWD));
  }

  // Publish control message
  orca_msgs::msg::Control control_msg;
  control_msg.header.stamp = now();
  control_msg.mode = mode_;
  control_msg.camera_tilt_pwm = tilt_to_pwm(tilt_);
  control_msg.brightness_pwm = brightness_to_pwm(brightness_);
  for (int i = 0; i < thruster_efforts.size(); ++i) {
    control_msg.thruster_pwm.push_back(effort_to_pwm(thruster_efforts[i]));
  }
  control_pub_->publish(control_msg);

  // Publish rviz marker message
  visualization_msgs::msg::MarkerArray markers_msg;
  for (int i = 0; i < thruster_efforts.size(); ++i) {
    int32_t action = thruster_efforts[i] == 0.0 ? visualization_msgs::msg::Marker::DELETE : visualization_msgs::msg::Marker::ADD;
    double scale = (THRUSTERS[i].ccw ? -thruster_efforts[i] : thruster_efforts[i]) / 5.0;
    double offset = scale > 0 ? -0.1 : 0.1;

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = THRUSTERS[i].frame_id;
    marker.header.stamp = now();
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

// Change operation mode
void OrcaBase::set_mode(uint8_t new_mode)
{
  // Stop all thrusters when we change modes
  efforts_.clear();

  if (is_z_hold_mode(new_mode)) {  // TODO move to keep station planner
    // Set target z
    z_setpoint_ = z_state_;
    z_controller_.set_target(z_setpoint_);
    RCLCPP_INFO(get_logger(), "hold z at %g", z_setpoint_);
  }

  if (is_yaw_hold_mode(new_mode)) {  // TODO move to keep station planner
    // Set target angle
    yaw_setpoint_ = yaw_state_;
    yaw_controller_.set_target(yaw_setpoint_);
    RCLCPP_INFO(get_logger(), "hold yaw at %g", yaw_setpoint_);
  }

  if (new_mode == orca_msgs::msg::Control::DISARMED) {
    // Turn off lights
    brightness_ = 0;
  }

  // Set the new mode
  mode_ = new_mode;
}

// New input from the gamepad
void OrcaBase::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  static sensor_msgs::msg::Joy prev_msg;
  prev_joy_time_ = msg->header.stamp;

  // Arm/disarm
  if (button_down(msg, prev_msg, joy_button_disarm_)) {
    RCLCPP_INFO(get_logger(), "disarmed");
    set_mode(orca_msgs::msg::Control::DISARMED);
  } else if (button_down(msg, prev_msg, joy_button_arm_)) {
    RCLCPP_INFO(get_logger(), "armed, manual");
    set_mode(orca_msgs::msg::Control::MANUAL);
  }

  // If we're disarmed, ignore everything else
  if (mode_ == orca_msgs::msg::Control::DISARMED) {
    prev_msg = *msg;
    return;
  }

  // Mode
  if (button_down(msg, prev_msg, joy_button_manual_)) {
    RCLCPP_INFO(get_logger(), "manual");
    set_mode(orca_msgs::msg::Control::MANUAL);
  } else if (button_down(msg, prev_msg, joy_button_hold_h_)) {
    if (imu_ready_) {
      set_mode(orca_msgs::msg::Control::HOLD_H);
    } else {
      RCLCPP_ERROR(get_logger(), "IMU not ready, can't hold yaw");
    }
  } else if (button_down(msg, prev_msg, joy_button_hold_d_)) {
    if (barometer_ready_) {
      set_mode(orca_msgs::msg::Control::HOLD_D);
    } else {
      RCLCPP_ERROR(get_logger(), "barometer not ready, can't hold z");
    }
  } else if (button_down(msg, prev_msg, joy_button_hold_hd_)) {
    if (imu_ready_ && barometer_ready_) {
      set_mode(orca_msgs::msg::Control::HOLD_HD);
    } else {
      RCLCPP_ERROR(get_logger(), "barometer and/or IMU not ready, can't hold yaw and z");
    }
  }

  // Yaw trim
  if (holding_yaw() && trim_down(msg, prev_msg, joy_axis_yaw_trim_)) {
    yaw_setpoint_ = msg->axes[joy_axis_yaw_trim_] > 0 ? yaw_setpoint_ + inc_yaw_ : yaw_setpoint_ - inc_yaw_;
    yaw_setpoint_ = norm_angle(yaw_setpoint_);
    yaw_controller_.set_target(yaw_setpoint_);
    RCLCPP_INFO(get_logger(), "hold yaw at %g", yaw_setpoint_);
  }

  // Z trim
  if (holding_z() && trim_down(msg, prev_msg, joy_axis_z_trim_)) {
    z_setpoint_ = msg->axes[joy_axis_z_trim_] > 0 ? z_setpoint_ + inc_z_ : z_setpoint_ - inc_z_;
    z_setpoint_ = clamp(z_setpoint_, Z_HOLD_MIN, Z_HOLD_MAX);
    z_controller_.set_target(z_setpoint_);
    RCLCPP_INFO(get_logger(), "hold z at %g", z_setpoint_);
  }

  // Camera tilt
  if (button_down(msg, prev_msg, joy_button_tilt_up_)) {
    tilt_ = clamp(tilt_ + inc_tilt_, TILT_MIN, TILT_MAX);
    RCLCPP_INFO(get_logger(), "tilt at %d", tilt_);
  } else if (button_down(msg, prev_msg, joy_button_tilt_down_)) {
    tilt_ = clamp(tilt_ - inc_tilt_, TILT_MIN, TILT_MAX);
    RCLCPP_INFO(get_logger(), "tilt at %d", tilt_);
  }

  // Lights
  if (button_down(msg, prev_msg, joy_button_bright_)) {
    brightness_ = clamp(brightness_ + inc_lights_, BRIGHTNESS_MIN, BRIGHTNESS_MAX);
    RCLCPP_INFO(get_logger(), "lights at %d", brightness_);
  } else if (button_down(msg, prev_msg, joy_button_dim_)) {
    brightness_ = clamp(brightness_ - inc_lights_, BRIGHTNESS_MIN, BRIGHTNESS_MAX);
    RCLCPP_INFO(get_logger(), "lights at %d", brightness_);
  }

  // Thrusters
  if (rov_mode()) {
    efforts_.forward = dead_band(msg->axes[joy_axis_forward_], input_dead_band_) * xy_gain_;
    if (!holding_yaw()) {
      efforts_.yaw = dead_band(msg->axes[joy_axis_yaw_], input_dead_band_) * yaw_gain_;
    }
    efforts_.strafe = dead_band(msg->axes[joy_axis_strafe_], input_dead_band_) * xy_gain_;
    if (!holding_z()) {
      efforts_.vertical = dead_band(msg->axes[joy_axis_vertical_], input_dead_band_) * vertical_gain_;
    }
  }

  prev_msg = *msg;
}

// Our main loop
void OrcaBase::spin_once()
{
  auto time_now = now();
  double dt = (time_now - prev_loop_time_).seconds();
  // TODO check for 0 or negative dt (side effect of rclcpp::Rate limitation)
  if (dt <= 0.001) return;
  prev_loop_time_ = time_now;

  // Check for communication problems
  if (rov_mode() && time_now - prev_joy_time_ > COMM_TIMEOUT_DISARM) {
    RCLCPP_ERROR(get_logger(), "lost contact with topside, disarming");
    set_mode(orca_msgs::msg::Control::DISARMED);
  }

  // Compute yaw effort
  if (holding_yaw()) {
    double effort = yaw_controller_.calc(yaw_state_, dt, 0);
    efforts_.yaw = dead_band(effort * stability_, yaw_pid_dead_band_);
  }

  // Compute z effort
  if (holding_z()) {
    double effort = z_controller_.calc(z_state_, dt, 0);
    efforts_.vertical = dead_band(effort * stability_, z_pid_dead_band_);
  }

  // Run a mission
  if (auv_mode()) {
    BaseMission::add_to_path(mission_estimated_path_, odom_local_);
    mission_estimated_pub_->publish(mission_estimated_path_);

    BaseMission::add_to_path(mission_ground_truth_path_, odom_ground_truth_);
    mission_ground_truth_pub_->publish(mission_ground_truth_path_);

    OrcaPose u_bar;
    if (mission_->advance(time_now, odom_local_, odom_plan_, u_bar)) {
      // u_bar in world frame => normalized efforts in body frame
      efforts_.from_acceleration(u_bar, odom_plan_.pose.yaw);

      BaseMission::add_to_path(mission_plan_path_, odom_plan_.pose);
      mission_plan_pub_->publish(mission_plan_path_);

      // TODO deadband?
      efforts_.forward = clamp(efforts_.forward * stability_, -1.0, 1.0);
      efforts_.strafe = clamp(efforts_.strafe * stability_, -1.0, 1.0);
      efforts_.vertical = clamp(-efforts_.vertical * stability_, -1.0, 1.0);
      efforts_.yaw = clamp(efforts_.yaw * stability_, -1.0, 1.0);
    } else {
      RCLCPP_INFO(get_logger(), "mission complete");
      set_mode(orca_msgs::msg::Control::MANUAL);
    }
  }

  // Publish controls for thrusters, lights and camera tilt
  publish_control();

  // Publish odometry
  publish_odom();
}

} // namespace orca_base

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Init node
  auto node = std::make_shared<orca_base::OrcaBase>();

  // TODO rclcpp::Rate doesn't honor /clock, possibly fixed in Dashing
  rclcpp::Rate r(orca_base::SPIN_RATE);
  while (rclcpp::ok()) {
    // Do our work
    node->spin_once();

    // Respond to incoming messages
    rclcpp::spin_some(node);

    // Wait
    r.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
