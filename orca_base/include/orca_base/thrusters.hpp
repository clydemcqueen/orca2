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

#ifndef ORCA_BASE__THRUSTERS_HPP_
#define ORCA_BASE__THRUSTERS_HPP_

#include <string>
#include <utility>
#include <vector>

#include "orca_base/base_context.hpp"
#include "orca_msgs/msg/control.hpp"
#include "orca_shared/mw/efforts.hpp"

namespace orca_base
{

struct Thruster
{
  std::string frame_id;   // URDF link frame id
  bool ccw;               // True if counterclockwise
  double forward;
  double strafe;
  double yaw;
  double vertical;
  double prev_effort;     // Most recent effort

  Thruster(
    std::string _frame_id, bool _ccw,
    double _forward, double _strafe, double _yaw, double _vertical)
  : frame_id{std::move(_frame_id)},
    ccw{_ccw},
    forward{_forward},
    strafe{_strafe},
    yaw{_yaw},
    vertical{_vertical},
    prev_effort{} {}

  int efforts_to_pwm(const BaseContext & cxt, const mw::Efforts & efforts, bool & saturated);
};

class Thrusters
{
  std::vector<Thruster> thrusters_;
  std::vector<int> prev_pwm_;

public:
  Thrusters();

  /**
   * Combine efforts (forward, strafe, vertical, yaw) to the 6 thruster PWM signals, and write to a control message
   *
   * @param efforts Efforts
   * @param xy_limit Limit forward + strafe efforts to this value
   * @param control_msg Write to this control message
   */
  void efforts_to_control(
    const BaseContext & cxt, const mw::Efforts & efforts,
    orca_msgs::msg::Control & control_msg);
};

}  // namespace orca_base

#endif  // ORCA_BASE__THRUSTERS_HPP_
