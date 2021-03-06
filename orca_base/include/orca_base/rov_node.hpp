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

#ifndef ORCA_BASE__ROV_NODE_HPP_
#define ORCA_BASE__ROV_NODE_HPP_

#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "orca_base/joystick.hpp"
#include "orca_base/rov_context.hpp"
#include "orca_base/pid.hpp"
#include "orca_base/thrusters.hpp"
#include "orca_msgs/action/mission.hpp"
#include "orca_msgs/msg/barometer.hpp"
#include "orca_msgs/msg/control.hpp"
#include "orca_msgs/msg/driver.hpp"
#include "orca_shared/monotonic.hpp"
#include "orca_shared/mw/mw.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace orca_base
{

//=============================================================================
// ROVNode
//=============================================================================

class ROVNode : public rclcpp::Node
{
  // Joystick assignments
  const int joy_axis_yaw_ = JOY_AXIS_LEFT_LR;
  const int joy_axis_forward_ = JOY_AXIS_LEFT_FB;
  const int joy_axis_strafe_ = JOY_AXIS_RIGHT_LR;
  const int joy_axis_vertical_ = JOY_AXIS_RIGHT_FB;
  const int joy_axis_z_trim_ = JOY_AXIS_TRIM_FB;

  const int joy_button_disarm_ = JOY_BUTTON_VIEW;
  const int joy_button_arm_ = JOY_BUTTON_MENU;
  const int joy_button_rov_ = JOY_BUTTON_A;
  const int joy_button_rov_hold_pressure_ = JOY_BUTTON_B;
  const int joy_button_auv_keep_station_ = JOY_BUTTON_X;
  const int joy_button_auv_random_ = JOY_BUTTON_Y;

  const int joy_button_tilt_down_ = JOY_BUTTON_LEFT_BUMPER;
  const int joy_button_tilt_up_ = JOY_BUTTON_RIGHT_BUMPER;
  const int joy_button_bright_ = JOY_BUTTON_LEFT_STICK;
  const int joy_button_dim_ = JOY_BUTTON_RIGHT_STICK;

  // Parameters and dynamics model
  ROVContext cxt_;

  // Timeouts will be set by parameters
  rclcpp::Duration baro_timeout_{0};
  rclcpp::Duration driver_timeout_{0};
  rclcpp::Duration joy_timeout_{0};
  std::chrono::milliseconds spin_period_{0};

  // Mode
  uint8_t mode_{orca_msgs::msg::Control::DISARMED};

  // Barometer state
  double pressure_{};

  // Joystick state
  sensor_msgs::msg::Joy joy_msg_;               // Most recent message

  // ROV operation
  double target_pressure_{};
  std::shared_ptr<pid::Controller> pressure_hold_pid_;

  // Outputs
  int tilt_{};                                  // Camera tilt
  int brightness_{};                            // Lights

  // Subscriptions
  rclcpp::Subscription<orca_msgs::msg::Barometer>::SharedPtr baro_sub_;
  rclcpp::Subscription<orca_msgs::msg::Driver>::SharedPtr driver_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  // Timer
  rclcpp::TimerBase::SharedPtr spin_timer_;

  // Thrusters
  Thrusters thrusters_;

  // Validate parameters
  void validate_parameters();

  // State testers
  bool holding_pressure() const;

  bool rov_mode() const;

  bool auv_mode() const;

  bool baro_ok(const rclcpp::Time & t) const;

  bool driver_ok(const rclcpp::Time & t) const;

  bool joy_ok(const rclcpp::Time & t) const;

  // Timer callback
  void spin_once();

  // Subscription callbacks
  void baro_callback(orca_msgs::msg::Barometer::SharedPtr msg);

  void goal_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg);

  void joy_callback(sensor_msgs::msg::Joy::SharedPtr msg, bool first);

  void driver_callback(orca_msgs::msg::Driver::SharedPtr msg);

  // Callback wrappers
  monotonic::Valid<ROVNode *, orca_msgs::msg::Barometer::SharedPtr>
  baro_cb_{this, &ROVNode::baro_callback};
  monotonic::Valid<ROVNode *, orca_msgs::msg::Driver::SharedPtr>
  driver_cb_{this, &ROVNode::driver_callback};
  monotonic::Monotonic<ROVNode *, sensor_msgs::msg::Joy::SharedPtr>
  joy_cb_{this, &ROVNode::joy_callback};

  // Publications
  rclcpp::Publisher<orca_msgs::msg::Control>::SharedPtr control_pub_;

  using MissionAction = orca_msgs::action::Mission;
  using MissionHandle = rclcpp_action::ClientGoalHandle<MissionAction>;

  // Action client
  rclcpp_action::Client<orca_msgs::action::Mission>::SharedPtr mission_client_;

  // Action client callbacks
  void goal_response_callback(std::shared_future<MissionHandle::SharedPtr> future);

  void feedback_callback(
    MissionHandle::SharedPtr,
    std::shared_ptr<const MissionAction::Feedback> feedback);

  void result_callback(const MissionHandle::WrappedResult & result);

  void rov_advance(const rclcpp::Time & stamp);

  void publish_control(const rclcpp::Time & msg_time, const mw::Efforts & efforts);

  void disarm(const rclcpp::Time & msg_time);

  void start_rov(const rclcpp::Time & msg_time);

  void start_hold_pressure(const rclcpp::Time & msg_time);

  enum class Mission
  {
    keep_station, go_to_pose, random_markers
  };

  void start_mission(
    const rclcpp::Time & msg_time, Mission mission,
    const geometry_msgs::msg::Pose & pose = geometry_msgs::msg::Pose{});

  void stop_mission();

public:
  ROVNode();

  ~ROVNode() override;
};

}  // namespace orca_base

#endif  // ORCA_BASE__ROV_NODE_HPP_
