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

#ifndef ORCA_DRIVER__DRIVER_NODE_HPP_
#define ORCA_DRIVER__DRIVER_NODE_HPP_

#include <string>
#include <vector>

#include "mraa/common.hpp"
#include "mraa/led.hpp"

#include "rclcpp/rclcpp.hpp"

#include "orca_driver/driver_context.hpp"
#include "orca_driver/maestro.hpp"
#include "orca_msgs/msg/control.hpp"
#include "orca_msgs/msg/driver.hpp"

namespace orca_driver
{

#define UP_LEDS
#ifdef UP_LEDS
// LEDs on the UP board
// https://github.com/intel-iot-devkit/mraa/blob/master/examples/platform/up2-leds.cpp
#define LED_READY_ON() led_ready_.setBrightness(led_ready_.readMaxBrightness() / 2)
#define LED_MISSION_ON() led_mission_.setBrightness(led_mission_.readMaxBrightness() / 2)
#define LED_PROBLEM_ON() led_problem_.setBrightness(led_problem_.readMaxBrightness() / 2)
#define LED_READY_OFF() led_ready_.setBrightness(0)
#define LED_MISSION_OFF() led_mission_.setBrightness(0)
#define LED_PROBLEM_OFF() led_problem_.setBrightness(0)
#else
#define LED_READY_ON()
#define LED_MISSION_ON()
#define LED_PROBLEM_ON()
#define LED_READY_OFF()
#define LED_MISSION_OFF()
#define LED_PROBLEM_OFF()
#endif

struct Thruster
{
  int channel_;
  bool reverse_;

  Thruster(int channel, bool reverse)
  : channel_{channel}, reverse_{reverse} {}
};

// DriverNode provides the interface between the Orca hardware and ROS.

class DriverNode : public rclcpp::Node
{
  // Parameters
  DriverContext cxt_;
  std::vector<Thruster> thrusters_;

  // Timeout, set by parameter
  rclcpp::Duration control_timeout_{0};

  // State
  maestro::Maestro maestro_;
  orca_msgs::msg::Driver driver_msg_;

  // Control message state
  rclcpp::Subscription<orca_msgs::msg::Control>::SharedPtr control_sub_;
  rclcpp::Time control_msg_time_;     // Set to now() when a message is received
  double control_msg_lag_{0};         // Difference between now() and msg.header.stamp

  // Timer
  rclcpp::TimerBase::SharedPtr spin_timer_;

  // Publication
  rclcpp::Publisher<orca_msgs::msg::Driver>::SharedPtr driver_pub_;

#ifdef UP_LEDS
  mraa::Led led_ready_{"yellow"};
  mraa::Led led_mission_{"green"};
  mraa::Led led_problem_{"red"};
#endif

  void validate_parameters();

  void set_status(uint8_t status);

  void set_thruster(const Thruster & thruster, uint16_t pwm);

  void control_callback(orca_msgs::msg::Control::SharedPtr msg);

  void timer_callback();

  bool read_battery();

  bool read_leak();

  bool read_temp();

  bool connect_controller();

  void all_stop();

  void abort();

public:
  DriverNode();

  ~DriverNode() override;
};

}  // namespace orca_driver

#endif  // ORCA_DRIVER__DRIVER_NODE_HPP_
