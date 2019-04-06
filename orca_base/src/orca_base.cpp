#include "orca_base/orca_base.hpp"
#include "orca_base/orca_pwm.hpp"

namespace orca_base {

// Limits
constexpr double DEPTH_HOLD_MIN = 0.05; // Hover just below the surface of the water
constexpr double DEPTH_HOLD_MAX = 50;   // Max depth is 100m, but provide a margin of safety

// Message publish rate in Hz
constexpr int SPIN_RATE = 10;

// Timeouts
constexpr int64_t COMM_ERROR_TIMEOUT_DISARM_NS = 5e+9;  // Disarm if we can't communicate with the topside
constexpr int64_t COMM_ERROR_TIMEOUT_SOS_NS = 1e+11;    // Panic if it's been too long

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

OrcaBase::OrcaBase():
  Node{"orca_base"},
  simulation_{true},
  mode_{orca_msgs::msg::Control::DISARMED},
  imu_ready_{false},
  barometer_ready_{false},
  tilt_{0},
  tilt_trim_button_previous_{false},
  brightness_{0},
  brightness_trim_button_previous_{false},
  prev_loop_time_{now()},
  depth_controller_{false, 0.1, 0, 0.05},
  yaw_controller_{true, 0.007, 0, 0}
{
  // TODO parameters
  joy_axis_yaw_ = 0;            // Left stick left/right; 1.0 is left and -1.0 is right
  joy_axis_forward_ = 1;        // Left stick up/down; 1.0 is forward and -1.0 is backward
  joy_axis_strafe_ = 3;         // Right stick left/right; 1.0 is left and -1.0 is right
  joy_axis_vertical_ = 4;       // Right stick up/down; 1.0 is ascend and -1.0 is descend
  joy_axis_yaw_trim_ = 6;       // Trim left/right; acts like 2 buttons; 1.0 for left and -1.0 for right
  joy_axis_vertical_trim_ = 7;  // Trim up/down; acts like 2 buttons; 1.0 for up and -1.0 for down

  joy_button_disarm_ = 6;       // View
  joy_button_arm_ = 7;          // Menu
  joy_button_manual_ = 0;       // A
  joy_button_hold_h_ = 2;       // X
  joy_button_hold_d_ = 1;       // B
  joy_button_hold_hd_ = 3;      // Y
  joy_button_tilt_down_ = 4;    // Left bumper
  joy_button_tilt_up_ = 5;      // Right bumper
  joy_button_bright_ = 9;       // Left stick
  joy_button_dim_ = 10;         // Right stick

  inc_yaw_ = M_PI/36;
  inc_depth_ = 0.1;
  inc_tilt_ = 5;
  inc_lights_ = 20;
  input_dead_band_ = 0.05f;     // Don't respond to tiny joystick movements
  yaw_pid_dead_band_ = 0.0005;
  depth_pid_dead_band_ = 0.002;
  xy_gain_ = 0.5;
  yaw_gain_ = 0.2;
  vertical_gain_ = 0.5;

  simulation_ = true;

  if (simulation_)
  {
    // The simulated IMU is not rotated
    RCLCPP_INFO(get_logger(), "Running in a simulation");
    imu_rotation_ = tf2::Matrix3x3::getIdentity();
  }
  else
  {
    // The actual IMU is rotated
    RCLCPP_INFO(get_logger(), "Running in real life");
    tf2::Matrix3x3 imu_orientation;
    imu_orientation.setRPY(-M_PI/2, -M_PI/2, 0);
    imu_rotation_ =  imu_orientation.inverse();
  }

  // Set up all subscriptions
  using std::placeholders::_1;
  baro_sub_ = create_subscription<orca_msgs::msg::Barometer>("/barometer", std::bind(&OrcaBase::baroCallback, this, _1));
  battery_sub_ = create_subscription<orca_msgs::msg::Battery>("/orca_driver/battery", std::bind(&OrcaBase::batteryCallback, this, _1));
  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", std::bind(&OrcaBase::goalCallback, this, _1));
  gps_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/gps", std::bind(&OrcaBase::gpsCallback, this, _1));
  ground_truth_sub_ = create_subscription<nav_msgs::msg::Odometry>("/ground_truth", std::bind(&OrcaBase::groundTruthCallback, this, _1));
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/imu/data", std::bind(&OrcaBase::imuCallback, this, _1));
  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>("/joy", std::bind(&OrcaBase::joyCallback, this, _1));
  leak_sub_ = create_subscription<orca_msgs::msg::Leak>("/orca_driver/leak", std::bind(&OrcaBase::leakCallback, this, _1));
  odom_local_sub_ = create_subscription<nav_msgs::msg::Odometry>("/camera1/filtered_odom", std::bind(&OrcaBase::odomLocalCallback, this, _1)); // TODO

  // Advertise all topics that we'll publish on
  control_pub_ = create_publisher<orca_msgs::msg::Control>("/orca_base/control", 1);
  odom_plan_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odometry/plan", 1);
  thrust_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("thrust_markers", 1);
  mission_plan_pub_ = create_publisher<nav_msgs::msg::Path>("mission_plan", 1);
  mission_estimated_pub_ = create_publisher<nav_msgs::msg::Path>("mission_estimated", 1);
  mission_ground_truth_pub_ = create_publisher<nav_msgs::msg::Path>("mission_ground_truth", 1);
}

// New barometer reading
void OrcaBase::baroCallback(const orca_msgs::msg::Barometer::SharedPtr baro_msg)
{
  if (!barometer_ready_)
  {
    // First depth reading: zero the depth
    depth_adjustment_ = baro_msg->depth;
    depth_state_ = 0;
    barometer_ready_ = true;
    RCLCPP_INFO(get_logger(), "Barometer ready, depth adjustment %g", depth_adjustment_);
  }
  else
  {
    depth_state_ = baro_msg->depth - depth_adjustment_;
  }
}

// New battery reading
void OrcaBase::batteryCallback(const orca_msgs::msg::Battery::SharedPtr battery_msg)
{
  if (battery_msg->low_battery)
  {
    RCLCPP_ERROR(get_logger(), "SOS! Low battery! %g volts", battery_msg->voltage);
    setMode(orca_msgs::msg::Control::SOS);
  }
}

// New 2D goal
void OrcaBase::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // TODO move some of this logic to setMode
  if (mode_ != orca_msgs::msg::Control::DISARMED && barometer_ready_ && imu_ready_)
  {
    // Start plan at current estimate
    odom_plan_.pose = odom_local_;
    odom_plan_.stopMotion();

    // Pull out yaw (kinda cumbersome)
    tf2::Quaternion goal_orientation;
    tf2::fromMsg(msg->pose.orientation, goal_orientation);
    double roll = 0, pitch = 0, goal_yaw = 0;
    tf2::Matrix3x3(goal_orientation).getRPY(roll, pitch, goal_yaw);

    OrcaPose goal_pose(msg->pose.position.x, msg->pose.position.y, odom_plan_.pose.z, goal_yaw);

    RCLCPP_INFO(get_logger(), "Start mission at (%g, %g, %g), goal is (%g, %g, %g), heading %g", odom_plan_.pose.x, odom_plan_.pose.y, odom_plan_.pose.z, goal_pose.x, goal_pose.y, goal_pose.z, goal_pose.yaw);

    mission_.reset(new SquareMission(get_logger()));
    if (mission_->init(now(), goal_pose, odom_plan_))
    {
      mission_plan_path_.header.stamp = now();
      mission_plan_path_.header.frame_id = "odom";
      mission_plan_path_.poses.clear();

      mission_estimated_path_.header.stamp = now();
      mission_estimated_path_.header.frame_id = "odom";
      mission_estimated_path_.poses.clear();

      mission_ground_truth_path_.header.stamp = now();
      mission_ground_truth_path_.header.frame_id = "odom";
      mission_ground_truth_path_.poses.clear();

      setMode(orca_msgs::msg::Control::MISSION);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Can't initialize mission; internal error");
    }
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "Can't start mission; possible reasons: disarmed, barometer not ready, IMU not ready");
  }
}

// New GPS reading
void OrcaBase::gpsCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  if (!gps_ready_)
  {
    gps_ready_ = true;
    RCLCPP_INFO(get_logger(), "GPS ready (%g, %g, %g)", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  }
}

// New ground truth message
void OrcaBase::groundTruthCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!ground_truth_ready_)
  {
    ground_truth_ready_ = true;
    RCLCPP_INFO(get_logger(), "Ground truth available");
  }

  odom_ground_truth_.fromMsg(*msg);
}

// New IMU reading
void OrcaBase::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
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

  if (!imu_ready_)
  {
    imu_ready_ = true;
    RCLCPP_INFO(get_logger(), "IMU ready, roll %4.2f pitch %4.2f yaw %4.2f", roll, pitch, yaw_state_);
  }
}

// Leak detector
void OrcaBase::leakCallback(const orca_msgs::msg::Leak::SharedPtr leak_msg)
{
  if (leak_msg->leak_detected)
  {
    RCLCPP_ERROR(get_logger(), "SOS! Leak detected!");
    setMode(orca_msgs::msg::Control::SOS);
  }
}

// Local odometry -- result from robot_localization, fusing all continuous sensors
void OrcaBase::odomLocalCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odom_local_.fromMsg(*msg);
}

void OrcaBase::publishOdom()
{
  nav_msgs::msg::Odometry odom_msg;

  odom_msg.header.stamp = now(); // TODO motion time
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";

  odom_plan_.toMsg(odom_msg);

  odom_plan_pub_->publish(odom_msg);
}

void OrcaBase::publishControl()
{
  // Combine joystick efforts to get thruster efforts.
  std::vector<double> thruster_efforts = {};
  for (int i = 0; i < THRUSTERS.size(); ++i)
  {
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
  for (int i = 0; i < thruster_efforts.size(); ++i)
  {
    control_msg.thruster_pwm.push_back(effort_to_pwm(thruster_efforts[i]));
  }
  control_pub_->publish(control_msg);

  // Publish rviz marker message
  visualization_msgs::msg::MarkerArray markers_msg;
  for (int i = 0; i < thruster_efforts.size(); ++i)
  {
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
void OrcaBase::setMode(uint8_t new_mode)
{
  // Stop all thrusters when we change modes
  efforts_.clear();

  if (depthHoldMode(new_mode)) // TODO move to keep station planner
  {
    // Set target depth
    depth_setpoint_ = depth_state_;
    depth_controller_.setTarget(depth_setpoint_);

    // Clear button state
    depth_trim_button_previous_ = false;
  }

  if (headingHoldMode(new_mode)) // TODO move to keep station planner
  {
    // Set target angle
    yaw_setpoint_ = yaw_state_;
    yaw_controller_.setTarget(yaw_setpoint_);

    // Clear button state
    yaw_trim_button_previous_ = false;
  }

  if (new_mode == orca_msgs::msg::Control::DISARMED)
  {
    // Turn off lights
    brightness_ = 0;
  }

  // TODO sos

  // Set the new mode
  mode_ = new_mode;
}

// New input from the gamepad
void OrcaBase::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
{
  // If we're in trouble, ignore the joystick
  if (mode_ == orca_msgs::msg::Control::SOS)
  {
    RCLCPP_INFO(get_logger(), "SOS, ignoring joystick");
    return;
  }

  // Arm/disarm
  if (joy_msg->buttons[joy_button_disarm_])
  {
    RCLCPP_INFO(get_logger(), "Disarmed");
    setMode(orca_msgs::msg::Control::DISARMED);
  }
  else if (joy_msg->buttons[joy_button_arm_])
  {
    RCLCPP_INFO(get_logger(), "Armed, manual");
    setMode(orca_msgs::msg::Control::MANUAL);
  }

  // If we're disarmed, ignore everything else
  if (mode_ == orca_msgs::msg::Control::DISARMED)
  {
    // RCLCPP_INFO(get_logger(), "Disarmed, ignoring further input");
    return;
  }

  // Mode
  if (joy_msg->buttons[joy_button_manual_])
  {
    RCLCPP_INFO(get_logger(), "Manual");
    setMode(orca_msgs::msg::Control::MANUAL);
  }
  else if (joy_msg->buttons[joy_button_hold_h_])
  {
    if (imu_ready_)
    {
      RCLCPP_INFO(get_logger(), "Hold heading");
      setMode(orca_msgs::msg::Control::HOLD_H);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "IMU not ready, can't hold heading");
    }
  }
  else if (joy_msg->buttons[joy_button_hold_d_])
  {
    if (barometer_ready_)
    {
      RCLCPP_INFO(get_logger(), "Hold depth");
      setMode(orca_msgs::msg::Control::HOLD_D);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Barometer not ready, can't hold depth");
    }
  }
  else if (joy_msg->buttons[joy_button_hold_hd_])
  {
    if (imu_ready_ && barometer_ready_)
    {
      RCLCPP_INFO(get_logger(), "Hold heading and depth");
      setMode(orca_msgs::msg::Control::HOLD_HD);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Barometer and/or IMU not ready, can't hold heading and depth");
    }
  }

  // Yaw trim
  if (joy_msg->axes[joy_axis_yaw_trim_] != 0.0 && !yaw_trim_button_previous_)
  {
    // Rising edge
    if ((holdingHeading()))
    {
      yaw_setpoint_ = joy_msg->axes[joy_axis_yaw_trim_] > 0.0 ? yaw_setpoint_ + inc_yaw_ : yaw_setpoint_ - inc_yaw_;
      yaw_controller_.setTarget(yaw_setpoint_);
    }

    yaw_trim_button_previous_ = true;
  }
  else if (joy_msg->axes[joy_axis_yaw_trim_] == 0.0 && yaw_trim_button_previous_)
  {
    // Falling edge
    yaw_trim_button_previous_ = false;
  }

  // Depth trim
  if (joy_msg->axes[joy_axis_vertical_trim_] != 0.0 && !depth_trim_button_previous_)
  {
    // Rising edge
    if (holdingDepth())
    {
      depth_setpoint_ = clamp(joy_msg->axes[joy_axis_vertical_trim_] < 0 ? depth_setpoint_ + inc_depth_ : depth_setpoint_ - inc_depth_,
        DEPTH_HOLD_MIN, DEPTH_HOLD_MAX);
      depth_controller_.setTarget(depth_setpoint_);
    }

    depth_trim_button_previous_ = true;
  }
  else if (joy_msg->axes[joy_axis_vertical_trim_] == 0.0 && depth_trim_button_previous_)
  {
    // Falling edge
    depth_trim_button_previous_ = false;
  }

  // Camera tilt
  if ((joy_msg->buttons[joy_button_tilt_up_] || joy_msg->buttons[joy_button_tilt_down_]) && !tilt_trim_button_previous_)
  {
    // Rising edge
    tilt_ = clamp(joy_msg->buttons[joy_button_tilt_up_] ? tilt_ + inc_tilt_ : tilt_ - inc_tilt_, TILT_MIN, TILT_MAX);
    tilt_trim_button_previous_ = true;
  }
  else if (!joy_msg->buttons[joy_button_tilt_up_] && !joy_msg->buttons[joy_button_tilt_down_] && tilt_trim_button_previous_)
  {
    // Falling edge
    tilt_trim_button_previous_ = false;
  }

  // Lights
  if ((joy_msg->buttons[joy_button_bright_] || joy_msg->buttons[joy_button_dim_]) && !brightness_trim_button_previous_)
  {
    // Rising edge
    brightness_ = clamp(joy_msg->buttons[joy_button_bright_] ? brightness_ + inc_lights_ : brightness_ - inc_lights_, BRIGHTNESS_MIN, BRIGHTNESS_MAX);
    brightness_trim_button_previous_ = true;
  }
  else if (!joy_msg->buttons[joy_button_bright_] && !joy_msg->buttons[joy_button_dim_] && brightness_trim_button_previous_)
  {
    // Falling edge
    brightness_trim_button_previous_ = false;
  }

  // Thrusters
  if (rovOperation())
  {
    efforts_.forward = dead_band(joy_msg->axes[joy_axis_forward_], input_dead_band_) * xy_gain_;
    if (!holdingHeading())
    {
      efforts_.yaw = dead_band(joy_msg->axes[joy_axis_yaw_], input_dead_band_) * yaw_gain_;
    }
    efforts_.strafe = dead_band(joy_msg->axes[joy_axis_strafe_], input_dead_band_) * xy_gain_;
    if (!holdingDepth())
    {
      efforts_.vertical = dead_band(joy_msg->axes[joy_axis_vertical_], input_dead_band_) * vertical_gain_;
    }
  }
}

// Our main loop
void OrcaBase::spinOnce()
{
  auto time_now = now();
  double dt = (time_now - prev_loop_time_).nanoseconds() / 1e+9;
  prev_loop_time_ = time_now;

  // Check for communication problems TODO re-enable
#if 0
  if (rovOperation() && time_now - ping_time_ > rclcpp::Duration(COMM_ERROR_TIMEOUT_DISARM_NS))
  {
    if (time_now - ping_time_ > rclcpp::Duration(COMM_ERROR_TIMEOUT_SOS_NS))
    {
      RCLCPP_ERROR(get_logger(), "SOS! Lost contact for way too long");
      setMode(orca_msgs::msg::Control::SOS);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Lost contact with topside; disarming");
      setMode(orca_msgs::msg::Control::DISARMED);
    }
  }
#endif

  // Compute yaw effort
  if (holdingHeading())
  {
    double effort = yaw_controller_.calc(yaw_state_, dt, 0);
    efforts_.yaw = dead_band(effort * stability_, yaw_pid_dead_band_);
  }

  // Compute depth effort
  if (holdingDepth())
  {
    double effort = depth_controller_.calc(depth_state_, dt, 0);
    efforts_.vertical = dead_band(-effort * stability_, depth_pid_dead_band_);
  }

  // Run a mission
  if (mode_ == orca_msgs::msg::Control::MISSION)
  {
    BaseMission::addToPath(mission_estimated_path_, odom_local_);
    mission_estimated_pub_->publish(mission_estimated_path_);

    BaseMission::addToPath(mission_ground_truth_path_, odom_ground_truth_);
    mission_ground_truth_pub_->publish(mission_ground_truth_path_);

    OrcaPose u_bar;
    if (mission_->advance(time_now, odom_local_, odom_plan_, u_bar))
    {
      // u_bar in world frame => normalized efforts in body frame
      efforts_.fromAcceleration(u_bar, odom_plan_.pose.yaw);

      BaseMission::addToPath(mission_plan_path_, odom_plan_.pose);
      mission_plan_pub_->publish(mission_plan_path_);

      // TODO deadband?
      efforts_.forward = clamp(efforts_.forward * stability_, -1.0, 1.0);
      efforts_.strafe = clamp(efforts_.strafe * stability_, -1.0, 1.0);
      efforts_.vertical = clamp(-efforts_.vertical * stability_, -1.0, 1.0);
      efforts_.yaw = clamp(efforts_.yaw * stability_, -1.0, 1.0);
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Mission complete");
      setMode(orca_msgs::msg::Control::MANUAL);
    }
  }

  // Publish controls for thrusters, lights and camera tilt
  publishControl();

  // Publish odometry
  publishOdom();
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

  RCLCPP_INFO(node->get_logger(), "Entering main loop");
  rclcpp::Rate r(orca_base::SPIN_RATE);
  while (rclcpp::ok())
  {
    // Do our work
    node->spinOnce();

    // Respond to incoming messages
    rclcpp::spin_some(node);

    // Wait
    r.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
