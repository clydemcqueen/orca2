#include "orca_base/base_node.hpp"
#include "orca_base/pwm.hpp"

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
// BaseNode
//=============================================================================

BaseNode::BaseNode() :
  Node{"base_node"},
  mode_{orca_msgs::msg::Control::DISARMED},
  imu_ready_{false},
  barometer_ready_{false},
  tilt_{0},
  brightness_{0},
  prev_loop_time_{now()},
  prev_joy_time_{now()}
{
  // Suppress IDE warnings
  (void) baro_sub_;
  (void) battery_sub_;
  (void) goal_sub_;
  (void) imu_sub_;
  (void) joy_sub_;
  (void) leak_sub_;
  (void) odom_sub_;

  // Get parameters
  cxt_.load_parameters(*this);

  if (cxt_.simulation_) {
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

  // PID controllers
  z_pid_ = std::make_shared<pid::Controller>(false, cxt_.rov_z_pid_kp_, cxt_.rov_z_pid_ki_, cxt_.rov_z_pid_kd_);
  yaw_pid_ = std::make_shared<pid::Controller>(true, cxt_.rov_yaw_pid_kp_, cxt_.rov_yaw_pid_ki_, cxt_.rov_yaw_pid_kd_);

  // Publications TODO rethink default topic names
  control_pub_ = create_publisher<orca_msgs::msg::Control>("/orca_base/control", 1);
  odom_plan_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odometry/plan", 1);
  thrust_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("thrust_markers", 1);
  mission_plan_pub_ = create_publisher<nav_msgs::msg::Path>("mission_plan", 1);
  mission_estimated_pub_ = create_publisher<nav_msgs::msg::Path>("mission_estimated", 1);

  // Callbacks
  using std::placeholders::_1;
  auto baro_cb = std::bind(&BaseNode::baro_callback, this, _1);
  auto battery_cb = std::bind(&BaseNode::battery_callback, this, _1);
  auto goal_cb = std::bind(&BaseNode::goal_callback, this, _1);
  auto imu_cb = std::bind(&BaseNode::imu_callback, this, _1);
  auto joy_cb = std::bind(&BaseNode::joy_callback, this, _1);
  auto leak_cb = std::bind(&BaseNode::leak_callback, this, _1);
  auto odom_cb = std::bind(&BaseNode::odom_callback, this, _1);

  // Subscriptions TODO rethink default topic names
  baro_sub_ = create_subscription<orca_msgs::msg::Barometer>("/barometer", baro_cb);
  battery_sub_ = create_subscription<orca_msgs::msg::Battery>("/orca_driver/battery", battery_cb);
  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", goal_cb);
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/imu/data", imu_cb);
  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>("/joy", joy_cb);
  leak_sub_ = create_subscription<orca_msgs::msg::Leak>("/orca_driver/leak", leak_cb);
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("filtered_odom", odom_cb);

  RCLCPP_INFO(get_logger(), "orca_base ready");
}

// New barometer reading
void BaseNode::baro_callback(const orca_msgs::msg::Barometer::SharedPtr msg)
{
  if (!barometer_ready_) {
    // First barometer reading: calibrate
    z_initial_ = -msg->depth;
    z_ = 0;
    barometer_ready_ = true;
    RCLCPP_INFO(get_logger(), "barometer ready, z adjustment %g", z_initial_);
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

// New 2D goal
void BaseNode::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // TODO move some of this logic to set_mode
  if (mode_ != orca_msgs::msg::Control::DISARMED && barometer_ready_ && imu_ready_) {
    // Start plan at current estimate
    plan_.pose = estimate_;
    plan_.stop_motion();

    // Pull out yaw (kinda cumbersome)
    tf2::Quaternion goal_orientation;
    tf2::fromMsg(msg->pose.orientation, goal_orientation);
    double roll = 0, pitch = 0, goal_yaw = 0;
    tf2::Matrix3x3(goal_orientation).getRPY(roll, pitch, goal_yaw);

    OrcaPose goal_pose(msg->pose.position.x, msg->pose.position.y, plan_.pose.z, goal_yaw);

    RCLCPP_INFO(get_logger(), "start mission at (%g, %g, %g), goal is (%g, %g, %g), yaw %g",
      plan_.pose.x, plan_.pose.y, plan_.pose.z, goal_pose.x, goal_pose.y, goal_pose.z, goal_pose.yaw);

    mission_ = std::make_shared<SquareMission>(get_logger());
    if (mission_->init(now(), goal_pose, plan_)) {
      plan_path_.header.stamp = now();
      plan_path_.header.frame_id = cxt_.map_frame_;
      plan_path_.poses.clear();

      estimate_path_.header.stamp = now();
      estimate_path_.header.frame_id = cxt_.map_frame_;
      estimate_path_.poses.clear();

      set_mode(orca_msgs::msg::Control::MISSION);
    } else {
      RCLCPP_ERROR(get_logger(), "can't initialize mission, internal error");
    }
  } else {
    RCLCPP_ERROR(get_logger(), "can't start mission, possible reasons: disarmed, barometer not ready, IMU not ready");
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

  if (!imu_ready_) {
    imu_ready_ = true;
    RCLCPP_INFO(get_logger(), "IMU ready, roll %4.2f pitch %4.2f yaw %4.2f", roll, pitch, yaw_);
  }
}

// Leak detector
void BaseNode::leak_callback(const orca_msgs::msg::Leak::SharedPtr msg)
{
  if (msg->leak_detected) {
    RCLCPP_ERROR(get_logger(), "leak detected, disarming");
    set_mode(orca_msgs::msg::Control::DISARMED);
  }
}

// New estimate available
void BaseNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  estimate_.from_msg(*msg);
}

void BaseNode::publish_odom()
{
  nav_msgs::msg::Odometry odom_msg;

  odom_msg.header.stamp = now(); // TODO motion time
  odom_msg.header.frame_id = cxt_.map_frame_;
  odom_msg.child_frame_id = cxt_.base_frame_;

  plan_.to_msg(odom_msg);

  odom_plan_pub_->publish(odom_msg);
}

void BaseNode::publish_control()
{
  // Combine joystick efforts to get thruster efforts.
  std::vector<double> thruster_efforts = {};
  for (int i = 0; i < THRUSTERS.size(); ++i) {
    // Clamp forward + strafe to xy_gain_
    double xy_effort = clamp(
      efforts_.forward * THRUSTERS[i].forward_factor + efforts_.strafe * THRUSTERS[i].strafe_factor,
      -cxt_.xy_gain_, cxt_.xy_gain_);

    // Clamp total thrust
    thruster_efforts.push_back(
      clamp(xy_effort + efforts_.yaw * THRUSTERS[i].yaw_factor + efforts_.vertical * THRUSTERS[i].vertical_factor,
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
    int32_t action =
      thruster_efforts[i] == 0.0 ? visualization_msgs::msg::Marker::DELETE : visualization_msgs::msg::Marker::ADD;
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
void BaseNode::set_mode(uint8_t new_mode)
{
  // Stop all thrusters when we change modes
  efforts_.all_stop();

  if (is_z_hold_mode(new_mode)) {
    z_pid_->set_target(z_);
    RCLCPP_INFO(get_logger(), "hold z at %g", z_pid_->target());
  }

  if (is_yaw_hold_mode(new_mode)) {
    yaw_pid_->set_target(yaw_);
    RCLCPP_INFO(get_logger(), "hold yaw at %g", yaw_pid_->target());
  }

  if (new_mode == orca_msgs::msg::Control::DISARMED) {
    // Turn off lights
    brightness_ = 0;
  }

  // Set the new mode
  mode_ = new_mode;
}

// New input from the gamepad
void BaseNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
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
    yaw_pid_->set_target(yaw_pid_->target() + msg->axes[joy_axis_yaw_trim_] > 0 ? cxt_.inc_yaw_ : -cxt_.inc_yaw_);
    RCLCPP_INFO(get_logger(), "hold yaw at %g", yaw_pid_->target());
  }

  // Z trim
  if (holding_z() && trim_down(msg, prev_msg, joy_axis_z_trim_)) {
    z_pid_->set_target(clamp(
      msg->axes[joy_axis_z_trim_] > 0 ? z_pid_->target() + cxt_.inc_z_ : z_pid_->target() - cxt_.inc_z_,
      Z_HOLD_MIN, Z_HOLD_MAX));
    RCLCPP_INFO(get_logger(), "hold z at %g", z_pid_->target());
  }

  // Camera tilt
  if (button_down(msg, prev_msg, joy_button_tilt_up_)) {
    tilt_ = clamp(tilt_ + cxt_.inc_tilt_, TILT_MIN, TILT_MAX);
    RCLCPP_INFO(get_logger(), "tilt at %d", tilt_);
  } else if (button_down(msg, prev_msg, joy_button_tilt_down_)) {
    tilt_ = clamp(tilt_ - cxt_.inc_tilt_, TILT_MIN, TILT_MAX);
    RCLCPP_INFO(get_logger(), "tilt at %d", tilt_);
  }

  // Lights
  if (button_down(msg, prev_msg, joy_button_bright_)) {
    brightness_ = clamp(brightness_ + cxt_.inc_lights_, BRIGHTNESS_MIN, BRIGHTNESS_MAX);
    RCLCPP_INFO(get_logger(), "lights at %d", brightness_);
  } else if (button_down(msg, prev_msg, joy_button_dim_)) {
    brightness_ = clamp(brightness_ - cxt_.inc_lights_, BRIGHTNESS_MIN, BRIGHTNESS_MAX);
    RCLCPP_INFO(get_logger(), "lights at %d", brightness_);
  }

  // Thrusters
  if (rov_mode()) {
    efforts_.forward = dead_band(msg->axes[joy_axis_forward_], cxt_.input_dead_band_) * cxt_.xy_gain_;
    if (!holding_yaw()) {
      efforts_.yaw = dead_band(msg->axes[joy_axis_yaw_], cxt_.input_dead_band_) * cxt_.yaw_gain_;
    }
    efforts_.strafe = dead_band(msg->axes[joy_axis_strafe_], cxt_.input_dead_band_) * cxt_.xy_gain_;
    if (!holding_z()) {
      efforts_.vertical = dead_band(msg->axes[joy_axis_vertical_], cxt_.input_dead_band_) * cxt_.vertical_gain_;
    }
  }

  prev_msg = *msg;
}

// Our main loop
void BaseNode::spin_once()
{
  auto time_now = now();
  double dt = (time_now - prev_loop_time_).seconds();
  // TODO check for 0 or negative dt (side effect of rclcpp::Rate limitation)
  if (dt <= 0.001) { return; }
  prev_loop_time_ = time_now;

  // Check for communication problems
  if (rov_mode() && time_now - prev_joy_time_ > COMM_TIMEOUT_DISARM) {
    RCLCPP_ERROR(get_logger(), "lost contact with topside, disarming");
    set_mode(orca_msgs::msg::Control::DISARMED);
  }

  // Compute yaw effort
  if (holding_yaw()) {
    efforts_.yaw = clamp(yaw_pid_->calc(yaw_, dt, 0) * stability_, -1.0, 1.0);
  }

  // Compute z effort
  if (holding_z()) {
    efforts_.vertical = clamp(z_pid_->calc(z_, dt, 0) * stability_, -1.0, 1.0);
  }

  // Run a mission
  if (auv_mode()) {
    BaseMission::add_to_path(estimate_path_, estimate_);
    mission_estimated_pub_->publish(estimate_path_);

    OrcaPose u_bar;
    if (mission_->advance(time_now, estimate_, plan_, u_bar)) {
      // u_bar in world frame => normalized efforts in body frame
      efforts_.from_acceleration(u_bar, plan_.pose.yaw);
      efforts_.forward = clamp(efforts_.forward * stability_, -1.0, 1.0);
      efforts_.strafe = clamp(efforts_.strafe * stability_, -1.0, 1.0);
      efforts_.vertical = clamp(-efforts_.vertical * stability_, -1.0, 1.0);
      efforts_.yaw = clamp(efforts_.yaw * stability_, -1.0, 1.0);

      BaseMission::add_to_path(plan_path_, plan_.pose);
      mission_plan_pub_->publish(plan_path_);
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
  auto node = std::make_shared<orca_base::BaseNode>();

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
