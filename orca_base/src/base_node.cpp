#include "orca_base/base_node.hpp"
#include "orca_base/pwm.hpp"

namespace orca_base {

//=============================================================================
// Constants
//=============================================================================

const double Z_HOLD_MAX = -0.05;  // Highest z hold
const double Z_HOLD_MIN = -50;    // Lowest z hold

const rclcpp::Duration JOY_TIMEOUT{1000000000};   // ROV: disarm if we lose communication
const rclcpp::Duration ODOM_TIMEOUT{1000000000};  // AUV: disarm if we lose odometry
const rclcpp::Duration BARO_TIMEOUT{1000000000};  // All modes: disarm if we lose barometer
const rclcpp::Duration IMU_TIMEOUT{1000000000};   // All modes: disarm if we lose IMU

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
// Possible time problems, particularly during simulations:
// -- msg.header.stamp might be 0
// -- msg.header.stamp might repeat over consecutive messages
// -- msg.header.stamp might go backwards over consecutive messages
// Could use macros or lambdas to do inversion-of-control in callbacks
//=============================================================================

bool valid(const rclcpp::Time &t)
{
  return t.nanoseconds() > 0;
}

bool bootstrap(const rclcpp::Time &msg, const rclcpp::Time &prev)
{
  return valid(msg) && !valid(prev);
}

bool good_dt(const rclcpp::Time &msg, const rclcpp::Time &prev)
{
  return msg > prev;
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
  tilt_{0},
  brightness_{0}
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

  // ROV PID controllers
  rov_z_pid_ = std::make_shared<pid::Controller>(false, cxt_.rov_z_pid_kp_, cxt_.rov_z_pid_ki_, cxt_.rov_z_pid_kd_);
  rov_yaw_pid_ = std::make_shared<pid::Controller>(true, cxt_.rov_yaw_pid_kp_, cxt_.rov_yaw_pid_ki_,
    cxt_.rov_yaw_pid_kd_);

  // Publications
  control_pub_ = create_publisher<orca_msgs::msg::Control>("control", 1);
  planned_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("planned_odom", 1);
  thrust_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("thrust_markers", 1);
  planned_path_pub_ = create_publisher<nav_msgs::msg::Path>("planned_path", 1);
  filtered_path_pub_ = create_publisher<nav_msgs::msg::Path>("filtered_path", 1);

  // Callbacks
  using std::placeholders::_1;
  auto baro_cb = std::bind(&BaseNode::baro_callback, this, _1);
  auto battery_cb = std::bind(&BaseNode::battery_callback, this, _1);
  auto goal_cb = std::bind(&BaseNode::goal_callback, this, _1);
  auto imu_cb = std::bind(&BaseNode::imu_callback, this, _1);
  auto joy_cb = std::bind(&BaseNode::joy_callback, this, _1);
  auto leak_cb = std::bind(&BaseNode::leak_callback, this, _1);
  auto odom_cb = std::bind(&BaseNode::odom_callback, this, _1);

  // Subscriptions
  baro_sub_ = create_subscription<orca_msgs::msg::Barometer>("barometer", baro_cb);
  battery_sub_ = create_subscription<orca_msgs::msg::Battery>("battery", battery_cb);
  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", goal_cb);
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/imu/data", imu_cb);
  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>("joy", joy_cb);
  leak_sub_ = create_subscription<orca_msgs::msg::Leak>("leak", leak_cb);
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("filtered_odom", odom_cb);

  RCLCPP_INFO(get_logger(), "orca_base ready");
}

// New barometer reading
void BaseNode::baro_callback(const orca_msgs::msg::Barometer::SharedPtr msg)
{
  // Ignore zero
  rclcpp::Time msg_time{msg->header.stamp};
  if (!valid(msg_time)) {
    return;
  }

  // Bootstrap
  if (bootstrap(msg_time, baro_time_)) {
    baro_time_ = msg_time;
    z_initial_ = -msg->depth;
    z_ = 0;
    RCLCPP_INFO(get_logger(), "barometer adjustment %g", z_initial_);
    return;
  }

  // Ignore dt <= 0
  if (good_dt(msg_time, baro_time_)) {
    z_ = -msg->depth - z_initial_;
    baro_time_ = msg_time;
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
  if (mode_ != orca_msgs::msg::Control::DISARMED && valid(odom_time_)) {
    // Start plan at current estimate
    planned_pose_.pose = filtered_pose_;
    planned_pose_.stop_motion();

    // Pull out yaw (kinda cumbersome)
    tf2::Quaternion goal_orientation;
    tf2::fromMsg(msg->pose.orientation, goal_orientation);
    double roll = 0, pitch = 0, goal_yaw = 0;
    tf2::Matrix3x3(goal_orientation).getRPY(roll, pitch, goal_yaw);

    OrcaPose goal_pose(msg->pose.position.x, msg->pose.position.y, planned_pose_.pose.z, goal_yaw);

    RCLCPP_INFO(get_logger(), "start mission at (%g, %g, %g), goal is (%g, %g, %g), yaw %g",
      planned_pose_.pose.x, planned_pose_.pose.y, planned_pose_.pose.z, goal_pose.x, goal_pose.y, goal_pose.z,
      goal_pose.yaw);

    mission_ = std::make_shared<SquareMission>(get_logger());
    if (mission_->init(odom_time_, goal_pose, planned_pose_)) {
      planned_path_.header.stamp = now();
      planned_path_.header.frame_id = cxt_.map_frame_;
      planned_path_.poses.clear();

      filtered_path_.header.stamp = now();
      filtered_path_.header.frame_id = cxt_.map_frame_;
      filtered_path_.poses.clear();

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
  // Ignore zero
  rclcpp::Time msg_time{msg->header.stamp};
  if (!valid(msg_time)) {
    return;
  }

  // Bootstrap
  if (bootstrap(msg_time, imu_time_)) {
    imu_time_ = msg_time;
    return;
  }

  // Ignore dt <= 0
  if (good_dt(msg_time, imu_time_)) {

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

    imu_time_ = msg_time;
  }
}

// New input from the gamepad
void BaseNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // Ignore zero
  rclcpp::Time msg_time{msg->header.stamp};
  if (!valid(msg_time)) {
    return;
  }

  // Bootstrap
  rclcpp::Time prev_time{joy_msg_.header.stamp};
  if (bootstrap(msg_time, prev_time)) {
    joy_msg_ = *msg;
    return;
  }

  // Ignore dt <= 0
  if (good_dt(msg_time, prev_time)) {

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
    } else if (button_down(msg, joy_msg_, joy_button_hold_h_)) {
      if (valid(imu_time_)) {
        set_mode(orca_msgs::msg::Control::HOLD_H);
      } else {
        RCLCPP_ERROR(get_logger(), "IMU not ready, can't hold yaw");
      }
    } else if (button_down(msg, joy_msg_, joy_button_hold_d_)) {
      if (valid(baro_time_)) {
        set_mode(orca_msgs::msg::Control::HOLD_D);
      } else {
        RCLCPP_ERROR(get_logger(), "barometer not ready, can't hold z");
      }
    } else if (button_down(msg, joy_msg_, joy_button_hold_hd_)) {
      if (valid(imu_time_) && valid(baro_time_)) {
        set_mode(orca_msgs::msg::Control::HOLD_HD);
      } else {
        RCLCPP_ERROR(get_logger(), "barometer and/or IMU not ready, can't hold yaw and z");
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
      double dt = (msg_time - prev_time).seconds();
      assert(dt > 0);

      efforts_.forward = dead_band(msg->axes[joy_axis_forward_], cxt_.input_dead_band_) * cxt_.xy_gain_;
      efforts_.strafe = dead_band(msg->axes[joy_axis_strafe_], cxt_.input_dead_band_) * cxt_.xy_gain_;

      if (holding_yaw()) {
        efforts_.yaw = clamp(rov_yaw_pid_->calc(yaw_, dt, 0) * stability_, -1.0, 1.0);
      } else {
        efforts_.yaw = dead_band(msg->axes[joy_axis_yaw_], cxt_.input_dead_band_) * cxt_.yaw_gain_;
      }

      if (holding_z()) {
        efforts_.vertical = clamp(rov_z_pid_->calc(z_, dt, 0) * stability_, -1.0, 1.0);
      } else {
        efforts_.vertical = dead_band(msg->axes[joy_axis_vertical_], cxt_.input_dead_band_) * cxt_.vertical_gain_;
      }
    }

    // Publish controls for thrusters, lights and camera tilt
    publish_control();

    joy_msg_ = *msg;
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

// New pose estimate available
void BaseNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Ignore zero
  rclcpp::Time msg_time{msg->header.stamp};
  if (!valid(msg_time)) {
    return;
  }

  // Bootstrap
  if (bootstrap(msg_time, odom_time_)) {
    odom_time_ = msg_time;
    return;
  }

  // Ignore dt <= 0
  if (good_dt(msg_time, odom_time_)) {

    filtered_pose_.from_msg(*msg);

    // AUV operation
    if (auv_mode()) {
      BaseMission::add_to_path(filtered_path_, filtered_pose_);
      filtered_path_pub_->publish(filtered_path_);

      OrcaPose u_bar;
      if (mission_->advance(msg_time, filtered_pose_, planned_pose_, u_bar)) {
        efforts_.from_acceleration(u_bar, planned_pose_.pose.yaw);
        efforts_.forward = clamp(efforts_.forward * stability_, -1.0, 1.0);
        efforts_.strafe = clamp(efforts_.strafe * stability_, -1.0, 1.0);
        efforts_.vertical = clamp(-efforts_.vertical * stability_, -1.0, 1.0);
        efforts_.yaw = clamp(efforts_.yaw * stability_, -1.0, 1.0);
        publish_control();

        // Publish plan for rviz TODO count subscribers
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = msg_time;
        odom_msg.header.frame_id = cxt_.map_frame_;
        odom_msg.child_frame_id = cxt_.base_frame_;
        planned_pose_.to_msg(odom_msg);
        planned_odom_pub_->publish(odom_msg);

        BaseMission::add_to_path(planned_path_, planned_pose_.pose);
        planned_path_pub_->publish(planned_path_);
      } else {
        RCLCPP_INFO(get_logger(), "mission complete");
        set_mode(orca_msgs::msg::Control::MANUAL);
      }
    }

    odom_time_ = msg_time;
  }
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

  // Publish rviz thrust markers TODO count subscribers
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
    rov_z_pid_->set_target(z_);
    RCLCPP_INFO(get_logger(), "hold z at %g", rov_z_pid_->target());
  }

  if (is_yaw_hold_mode(new_mode)) {
    rov_yaw_pid_->set_target(yaw_);
    RCLCPP_INFO(get_logger(), "hold yaw at %g", rov_yaw_pid_->target());
  }

  if (new_mode == orca_msgs::msg::Control::DISARMED) {
    // Turn off lights
    brightness_ = 0;
  }

  // Set the new mode
  mode_ = new_mode;
}

void BaseNode::spin_once()
{
  // Ignore 0
  auto spin_time = now();
  if (!valid(spin_time)) {
    return;
  }

  rclcpp::Time joy_time{joy_msg_.header.stamp};
  if (rov_mode() && valid(joy_time) && spin_time - joy_time > JOY_TIMEOUT) {
    RCLCPP_ERROR(get_logger(), "lost joystick, disarming");
    set_mode(orca_msgs::msg::Control::DISARMED);
  }

  if (auv_mode() && valid(odom_time_) && spin_time - odom_time_ > ODOM_TIMEOUT) {
    RCLCPP_ERROR(get_logger(), "lost odometry, disarming");
    set_mode(orca_msgs::msg::Control::DISARMED);
  }

  if (valid(baro_time_) && spin_time - baro_time_ > BARO_TIMEOUT) {
    RCLCPP_ERROR(get_logger(), "lost barometer, disarming");
    set_mode(orca_msgs::msg::Control::DISARMED);
  }

  if (valid(imu_time_) && spin_time - imu_time_ > IMU_TIMEOUT) {
    RCLCPP_ERROR(get_logger(), "lost IMU, disarming");
    set_mode(orca_msgs::msg::Control::DISARMED);
  }
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

  // rclcpp::Rate doesn't honor /clock in Crystal -- loop will run at ~constant wall speed
  rclcpp::Rate r(100);
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
