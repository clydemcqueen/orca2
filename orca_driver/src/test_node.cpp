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

#include <memory>

#include "orca_msgs/msg/control.hpp"
#include "rclcpp/rclcpp.hpp"

namespace orca_driver
{
//=============================================================================
// TestNode
// Test Orca hardware by sending control messages
//=============================================================================

class TestNode : public rclcpp::Node
{
  const int THRUST_DIFF = 50;

  rclcpp::TimerBase::SharedPtr spin_timer_;
  rclcpp::Publisher<orca_msgs::msg::Control>::SharedPtr control_pub_;

  void spin_once()
  {
    orca_msgs::msg::Control msg;
    msg.header.stamp = now();
    msg.odom_lag = 0.0;
    msg.mode = orca_msgs::msg::Control::ROV;
    msg.camera_tilt_pwm = orca_msgs::msg::Control::TILT_0;
    msg.brightness_pwm = orca_msgs::msg::Control::LIGHTS_OFF;

    // Rotate through all 6 thrusters, and send fwd and rev signals
    // Each thruster gets 5s, so a cycle is 30s
    int cycle = msg.header.stamp.sec % 30;
    int thruster = cycle / 5;
    static int prev_thruster = -1;
    if (thruster != prev_thruster) {
      RCLCPP_INFO(get_logger(), "test thruster %d, fwd, rev, stop", thruster + 1);
      prev_thruster = thruster;
    }
    int pwm;
    switch (cycle % 5) {
      case 1:
        pwm = orca_msgs::msg::Control::THRUST_STOP + THRUST_DIFF;
        break;
      case 3:
        pwm = orca_msgs::msg::Control::THRUST_STOP - THRUST_DIFF;
        break;
      default:
        pwm = orca_msgs::msg::Control::THRUST_STOP;
        break;
    }

    msg.thruster_pwm.fr_1 = (thruster == 0) ? pwm : orca_msgs::msg::Control::THRUST_STOP;
    msg.thruster_pwm.fl_2 = (thruster == 1) ? pwm : orca_msgs::msg::Control::THRUST_STOP;
    msg.thruster_pwm.rr_3 = (thruster == 2) ? pwm : orca_msgs::msg::Control::THRUST_STOP;
    msg.thruster_pwm.rl_4 = (thruster == 3) ? pwm : orca_msgs::msg::Control::THRUST_STOP;
    msg.thruster_pwm.vr_5 = (thruster == 4) ? pwm : orca_msgs::msg::Control::THRUST_STOP;
    msg.thruster_pwm.vl_6 = (thruster == 5) ? pwm : orca_msgs::msg::Control::THRUST_STOP;

    control_pub_->publish(msg);
  }

public:
  TestNode()
  : Node{"self_test"}
  {
    // Suppress IDE warning
    (void) spin_timer_;

    // Publish control messages
    control_pub_ = create_publisher<orca_msgs::msg::Control>("control", 1);

    // Spin timer
    using namespace std::chrono_literals;
    spin_timer_ = create_wall_timer(100ms, std::bind(&TestNode::spin_once, this));
  }

  ~TestNode() override
  {
  }
};

}  // namespace orca_driver

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
  auto node = std::make_shared<orca_driver::TestNode>();

  // Spin
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
