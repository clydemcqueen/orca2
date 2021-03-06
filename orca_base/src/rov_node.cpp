// Copyright (c) 2020, Clyde McQueen.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "orca_base/rov_node.hpp"

#include <iomanip>
#include <memory>

#include "orca_shared/pwm.hpp"
#include "orca_base/thrusters.hpp"

namespace orca_base
{

//=============================================================================
// Utilities
//=============================================================================

// Sense a button down event
bool button_down(
  const sensor_msgs::msg::Joy::SharedPtr & curr, const sensor_msgs::msg::Joy & prev,
  int button)
{
  return curr->buttons[button] && !prev.buttons[button];
}

// Sense a trim down event
bool trim_down(
  const sensor_msgs::msg::Joy::SharedPtr & curr, const sensor_msgs::msg::Joy & prev,
  int axis)
{
  return curr->axes[axis] && !prev.axes[axis];
}

constexpr bool is_hold_pressure_mode(uint8_t mode)
{
  using orca_msgs::msg::Control;
  return mode == Control::ROV_HOLD_PRESSURE;
}

constexpr bool is_rov_mode(uint8_t mode)
{
  using orca_msgs::msg::Control;
  return mode == Control::ROV || mode == Control::ROV_HOLD_PRESSURE;
}

constexpr bool is_auv_mode(uint8_t mode)
{
  using orca_msgs::msg::Control;
  return mode == Control::AUV;
}

//=============================================================================
// ROVNode
//=============================================================================

constexpr int QUEUE_SIZE = 10;

ROVNode::ROVNode()
: Node{"rov_node"}
{
  // Suppress IDE warnings
  (void) baro_sub_;
  (void) goal_sub_;
  (void) joy_sub_;
  (void) spin_timer_;
  (void) driver_sub_;

  // Get parameters, this will immediately call validate_parameters()
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), cxt_, n, t, d)
  CXT_MACRO_INIT_PARAMETERS(ROV_NODE_ALL_PARAMS, validate_parameters)

  // Register parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(cxt_, n, t)
  CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), ROV_NODE_ALL_PARAMS, validate_parameters)

  // Log parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_INFO, get_logger(), cxt_, n, t, d)
  ROV_NODE_ALL_PARAMS

  // Check that all command line parameters are defined
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_CHECK_CMDLINE_PARAMETER(n, t, d)
  CXT_MACRO_CHECK_CMDLINE_PARAMETERS((*this), ROV_NODE_ALL_PARAMS)

  // Publications
  control_pub_ = create_publisher<orca_msgs::msg::Control>("control", QUEUE_SIZE);

  // Monotonic subscriptions
  baro_sub_ = create_subscription<orca_msgs::msg::Barometer>(
    "barometer", QUEUE_SIZE, [this](const orca_msgs::msg::Barometer::SharedPtr msg) -> void
    {this->baro_cb_.call(msg);});
  driver_sub_ = create_subscription<orca_msgs::msg::Driver>(
    "driver_status", QUEUE_SIZE, [this](const orca_msgs::msg::Driver::SharedPtr msg) -> void
    {this->driver_cb_.call(msg);});
  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "joy", QUEUE_SIZE, [this](const sensor_msgs::msg::Joy::SharedPtr msg) -> void
    {this->joy_cb_.call(msg);});

  using namespace std::placeholders;

  // Other subscriptions
  auto goal_cb = std::bind(&ROVNode::goal_callback, this, _1);
  goal_sub_ =
    create_subscription<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", QUEUE_SIZE,
      goal_cb);

  // Mission action client
  mission_client_ = rclcpp_action::create_client<MissionAction>(
    get_node_base_interface(),
    get_node_graph_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "mission");

  RCLCPP_INFO(get_logger(), "rov_node ready");
}

ROVNode::~ROVNode()
{
  // Disarm! This will stop an active mission and stop the thrusters
  disarm(now());
}

void ROVNode::validate_parameters()
{
  // Update timeouts
  baro_timeout_ = rclcpp::Duration{RCL_MS_TO_NS(cxt_.timeout_baro_ms_)};
  driver_timeout_ = rclcpp::Duration{RCL_MS_TO_NS(cxt_.timeout_driver_ms_)};
  joy_timeout_ = rclcpp::Duration{RCL_MS_TO_NS(cxt_.timeout_joy_ms_)};
  spin_period_ = std::chrono::milliseconds{cxt_.timer_period_ms_};

  // Create ROV PID controller
  pressure_hold_pid_ =
    std::make_shared<pid::Controller>(false, cxt_.rov_pressure_pid_kp_, cxt_.rov_pressure_pid_ki_,
      cxt_.rov_pressure_pid_kd_, cxt_.rov_pressure_pid_i_max_);

  // Set target!
  if (holding_pressure()) {
    pressure_hold_pid_->set_target(target_pressure_);
  }

  // [Re-]start loop
  // Loop will run at ~constant wall speed, switch to ros_timer when it exists
  spin_timer_ = create_wall_timer(spin_period_, std::bind(&ROVNode::spin_once, this));

  cxt_.log_info(get_logger());
}

bool ROVNode::holding_pressure() const {return is_hold_pressure_mode(mode_);}

bool ROVNode::rov_mode() const {return is_rov_mode(mode_);}

bool ROVNode::auv_mode() const {return is_auv_mode(mode_);}

bool ROVNode::baro_ok(const rclcpp::Time & t) const
{
  return baro_cb_.receiving() && t - baro_cb_.prev() < baro_timeout_;
}

bool ROVNode::driver_ok(const rclcpp::Time & t) const
{
  return driver_cb_.receiving() && t - driver_cb_.prev() < driver_timeout_;
}

bool ROVNode::joy_ok(const rclcpp::Time & t) const
{
  return joy_cb_.receiving() && t - joy_cb_.prev() < joy_timeout_;
}

// New barometer reading
void ROVNode::baro_callback(const orca_msgs::msg::Barometer::SharedPtr msg)
{
  pressure_ = msg->pressure;
}

void ROVNode::driver_callback(const orca_msgs::msg::Driver::SharedPtr msg)
{
  if ((rov_mode() || auv_mode()) && !(msg->status == orca_msgs::msg::Driver::STATUS_OK ||
    msg->status == orca_msgs::msg::Driver::STATUS_OK_MISSION))
  {
    RCLCPP_ERROR(get_logger(), "driver problem, disarm");
    disarm(msg->header.stamp);
  }
}

// Timer callback
void ROVNode::spin_once()
{
  // Ignore 0
  auto spin_time = now();
  if (spin_time.nanoseconds() <= 0) {
    return;
  }

  // If the driver stopped sending status messages, disarm
  if ((rov_mode() || auv_mode()) && !driver_ok(spin_time)) {
    RCLCPP_ERROR(get_logger(), "lost driver messages, disarm");
    disarm(spin_time);
  }

  // Various timeouts
  if (rov_mode() && !joy_ok(spin_time)) {
    RCLCPP_ERROR(get_logger(), "lost joystick during ROV operation, disarming");
    disarm(spin_time);
  }

  if (holding_pressure() && !baro_ok(spin_time)) {
    RCLCPP_ERROR(get_logger(), "lost barometer while holding pressure, disarming");
    disarm(spin_time);
  }
}

// Start a mission to move to a particular goal -- called by rviz2
void ROVNode::goal_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (rov_mode()) {
    RCLCPP_INFO(get_logger(), "goal accepted");
    msg->pose.position.z = cxt_.planner_target_z_;
    start_mission(msg->header.stamp, Mission::go_to_pose, msg->pose);
  } else {
    RCLCPP_ERROR(get_logger(), "cannot start mission from rviz2, armed?");
  }
}

// New input from the gamepad
void ROVNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg, bool first)
{
  if (!first) {
    // Arm/disarm
    if (button_down(msg, joy_msg_, joy_button_disarm_)) {
      disarm(msg->header.stamp);
    } else if (button_down(msg, joy_msg_, joy_button_arm_)) {
      start_rov(msg->header.stamp);
    }

    // If we're disarmed, ignore everything else
    if (mode_ == orca_msgs::msg::Control::DISARMED) {
      joy_msg_ = *msg;
      return;
    }

    // Mode
    if (button_down(msg, joy_msg_, joy_button_rov_)) {
      start_rov(msg->header.stamp);
    } else if (button_down(msg, joy_msg_, joy_button_rov_hold_pressure_)) {
      if (baro_ok(msg->header.stamp)) {
        start_hold_pressure(msg->header.stamp);
      } else {
        RCLCPP_ERROR(get_logger(), "barometer not ready, cannot hold pressure");
      }
    } else if (button_down(msg, joy_msg_, joy_button_auv_keep_station_)) {
      start_mission(msg->header.stamp, Mission::keep_station);
    } else if (button_down(msg, joy_msg_, joy_button_auv_random_)) {
      start_mission(msg->header.stamp, Mission::random_markers);
    }

    // Z trim
    if (holding_pressure() && trim_down(msg, joy_msg_, joy_axis_z_trim_)) {
      target_pressure_ +=
        (msg->axes[joy_axis_z_trim_] > 0) ? -cxt_.inc_pressure_ : cxt_.inc_pressure_;
      pressure_hold_pid_->set_target(target_pressure_);
      RCLCPP_INFO(get_logger(), "hold pressure at %g", target_pressure_);
    }

    // Camera tilt
    if (button_down(msg, joy_msg_, joy_button_tilt_up_)) {
      tilt_ = orca::clamp(tilt_ + cxt_.inc_tilt_, orca::TILT_MIN, orca::TILT_MAX);
      RCLCPP_INFO(get_logger(), "tilt at %d", tilt_);
    } else if (button_down(msg, joy_msg_, joy_button_tilt_down_)) {
      tilt_ = orca::clamp(tilt_ - cxt_.inc_tilt_, orca::TILT_MIN, orca::TILT_MAX);
      RCLCPP_INFO(get_logger(), "tilt at %d", tilt_);
    }

    // Lights
    if (button_down(msg, joy_msg_, joy_button_bright_)) {
      brightness_ =
        orca::clamp(brightness_ + cxt_.inc_lights_, orca::BRIGHTNESS_MIN, orca::BRIGHTNESS_MAX);
      RCLCPP_INFO(get_logger(), "lights at %d", brightness_);
    } else if (button_down(msg, joy_msg_, joy_button_dim_)) {
      brightness_ =
        orca::clamp(brightness_ - cxt_.inc_lights_, orca::BRIGHTNESS_MIN, orca::BRIGHTNESS_MAX);
      RCLCPP_INFO(get_logger(), "lights at %d", brightness_);
    }

    // Thrusters
    if (rov_mode()) {
      rov_advance(msg->header.stamp);
    }
  }

  joy_msg_ = *msg;
}

void ROVNode::goal_response_callback(std::shared_future<MissionHandle::SharedPtr> future)
{
  const auto & goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "mission rejected");
  } else {
    RCLCPP_INFO(get_logger(), "mission accepted");
    mode_ = orca_msgs::msg::Control::AUV;
  }
}

void ROVNode::feedback_callback(
  MissionHandle::SharedPtr,
  const std::shared_ptr<const MissionAction::Feedback> feedback)
{
  RCLCPP_INFO(get_logger(), "feedback: %d out of %d", feedback->targets_completed,
    feedback->targets_total);
}

void ROVNode::result_callback(const MissionHandle::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "mission succeeded");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(get_logger(), "mission aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(get_logger(), "mission canceled");
      return;
    default:
      RCLCPP_ERROR(get_logger(), "unknown result code");
      return;
  }

  disarm(now());
}

void ROVNode::rov_advance(const rclcpp::Time & stamp)
{
  double dt = joy_cb_.dt();

  mw::Efforts efforts;
  if (cxt_.mode_ == 0) {
    efforts.forward(
      orca::dead_band(joy_msg_.axes[joy_axis_forward_], cxt_.input_dead_band_) * cxt_.xy_gain_);
    efforts.strafe(
      orca::dead_band(joy_msg_.axes[joy_axis_strafe_], cxt_.input_dead_band_) * cxt_.xy_gain_);
    efforts.yaw(
      orca::dead_band(joy_msg_.axes[joy_axis_yaw_], cxt_.input_dead_band_) * cxt_.yaw_gain_);

    if (holding_pressure()) {
      auto accel_z = cxt_.hover_accel_z();
      // RCLCPP_INFO(get_logger(), "vert accel %g", accel_z);
      if (cxt_.rov_pid_enabled_) {
        accel_z -= pressure_hold_pid_->calc(pressure_, dt);
      }
      // RCLCPP_INFO(get_logger(), "vert accel %g", accel_z);
      efforts.vertical(cxt_.accel_to_effort_z(accel_z));
    } else {
      efforts.vertical(orca::dead_band(joy_msg_.axes[joy_axis_vertical_], cxt_.input_dead_band_) *
        cxt_.vertical_gain_);
    }
  } else if (cxt_.mode_ == 1) {
    // Test mode: 10% forward, etc.
    efforts.forward(0.1);
  } else if (cxt_.mode_ == 2) {
    efforts.forward(-0.1);
  } else if (cxt_.mode_ == 3) {
    efforts.strafe(0.1);
  } else if (cxt_.mode_ == 4) {
    efforts.strafe(-0.1);
  } else if (cxt_.mode_ == 5) {
    efforts.yaw(0.1);
  } else if (cxt_.mode_ == 6) {
    efforts.yaw(-0.1);
  } else if (cxt_.mode_ == 7) {
    efforts.vertical(0.1);
  } else if (cxt_.mode_ == 8) {
    efforts.vertical(-0.1);
  }

  publish_control(stamp, efforts);
}

void ROVNode::publish_control(const rclcpp::Time & msg_time, const mw::Efforts & efforts)
{
  orca_msgs::msg::Control control_msg;
  control_msg.header.stamp = msg_time;
  control_msg.header.frame_id = cxt_.base_frame_;  // Control is expressed in the base frame

  // Diagnostics
  control_msg.mode = mode_;
  control_msg.target_pressure = target_pressure_;
  control_msg.efforts = efforts.to_msg();
  control_msg.odom_lag = (now() - msg_time).seconds();

  // Control
  control_msg.camera_tilt_pwm = orca::tilt_to_pwm(tilt_);
  control_msg.brightness_pwm = orca::brightness_to_pwm(brightness_);
  thrusters_.efforts_to_control(cxt_, efforts, control_msg);

  control_pub_->publish(control_msg);
}

void ROVNode::disarm(const rclcpp::Time & msg_time)
{
  stop_mission();
  mode_ = orca_msgs::msg::Control::DISARMED;

  // joy_callback will no longer call publish_control
  // Call it once to let driver_node know we're disarmed
  publish_control(msg_time, {});

  RCLCPP_INFO(get_logger(), "disarmed");
}

void ROVNode::start_rov(const rclcpp::Time & msg_time)
{
  stop_mission();
  mode_ = orca_msgs::msg::Control::ROV;

  // joy_callback will call publish_control, so we don't have to
  // publish_control(msg_time, {});

  RCLCPP_INFO(get_logger(), "manual");
}

void ROVNode::start_hold_pressure(const rclcpp::Time & msg_time)
{
  stop_mission();
  target_pressure_ = pressure_;
  pressure_hold_pid_->set_target(target_pressure_);
  mode_ = orca_msgs::msg::Control::ROV_HOLD_PRESSURE;

  // joy_callback will call publish_control, so we don't have to
  // publish_control(msg_time, {});

  RCLCPP_INFO(get_logger(), "hold pressure at %g", pressure_);
}

void ROVNode::start_mission(
  const rclcpp::Time & msg_time, Mission mission,
  const geometry_msgs::msg::Pose & pose)
{
  // Disarm to avoid conflicts with auv_node
  disarm(msg_time);

  // Wait for action server
  if (!mission_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(get_logger(), "mission action server not responding, is auv_node running?");
    return;
  }

  auto goal_msg = MissionAction::Goal();
  switch (mission) {
    case Mission::keep_station:
      goal_msg.mission_info = "ROV keep station";
      goal_msg.target_type = MissionAction::Goal::TARGET_POSE;
      goal_msg.keep_station = true;
      RCLCPP_INFO(get_logger(), "keeping station at current pose");
      break;
    case Mission::go_to_pose:
      goal_msg.mission_info = "ROV go to pose";
      goal_msg.target_type = MissionAction::Goal::TARGET_POSE;
      goal_msg.poses.push_back(pose);
      RCLCPP_INFO(get_logger(), "go to pose");
      break;
    case Mission::random_markers:
      goal_msg.mission_info = "ROV random markers";
      goal_msg.random = true;
      RCLCPP_INFO(get_logger(), "visit all markers in a random order");
      break;
  }

  using namespace std::placeholders;
  auto send_goal_options = rclcpp_action::Client<MissionAction>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(&ROVNode::goal_response_callback, this, _1);
  send_goal_options.feedback_callback = std::bind(&ROVNode::feedback_callback, this, _1, _2);
  send_goal_options.result_callback = std::bind(&ROVNode::result_callback, this, _1);

  // Send goal to auv_node
  auto goal_handle_future = mission_client_->async_send_goal(goal_msg, send_goal_options);
}

// Don't change mode_, let caller do that
void ROVNode::stop_mission()
{
  if (auv_mode()) {
    mission_client_->async_cancel_all_goals();
    RCLCPP_INFO(get_logger(), "canceling all goals");
  }
}

}  // namespace orca_base

//=============================================================================
// Main
//=============================================================================

int main(int argc, char ** argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Init node
  auto node = std::make_shared<orca_base::ROVNode>();

  // Set logger level
  auto result =
    rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);
  (void) result;

  // Spin node
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
