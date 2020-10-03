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

#include "orca_base/thrusters.hpp"
#include "orca_shared/pwm.hpp"

namespace orca_base
{

int Thruster::efforts_to_pwm(const BaseContext & cxt, const mw::Efforts & efforts, bool & saturated)
{
  double combined_effort = efforts.forward() * forward + efforts.strafe() * strafe;

  // Clamp forward + strafe to xy_limit
  if (combined_effort > cxt.xy_limit_) {
    combined_effort = cxt.xy_limit_;
    saturated = true;
  } else if (combined_effort < -cxt.xy_limit_) {
    combined_effort = -cxt.xy_limit_;
    saturated = true;
  }

  combined_effort += efforts.yaw() * yaw;

  // Clamp forward + strafe + yaw to max values
  if (combined_effort > orca::THRUST_FULL_FWD) {
    combined_effort = orca::THRUST_FULL_FWD;
    saturated = true;
  } else if (combined_effort < orca::THRUST_FULL_REV) {
    combined_effort = orca::THRUST_FULL_REV;
    saturated = true;
  }

  double vertical_effort = efforts.vertical() * vertical;

  // Clamp vertical effort to max values
  if (vertical_effort > orca::THRUST_FULL_FWD) {
    vertical_effort = orca::THRUST_FULL_FWD;
    saturated = true;
  } else if (vertical_effort < orca::THRUST_FULL_REV) {
    vertical_effort = orca::THRUST_FULL_REV;
    saturated = true;
  }

  // Vertical effort is independent from the rest, no need to clamp
  double effort = combined_effort + vertical_effort;

  // Protect the thruster:
  // -- limit change to +/- max_change
  // -- don't reverse thruster, i.e., stop at 0.0
  effort = orca::clamp(effort,
      prev_effort - cxt.thruster_accel_limit_,
      prev_effort + cxt.thruster_accel_limit_);
  if (effort < 0 && prev_effort > 0 || effort > 0 && prev_effort < 0) {
    effort = 0;
  }

  prev_effort = effort;

  return orca::effort_to_pwm(cxt.mdl_thrust_dz_pwm_, effort);
}

Thrusters::Thrusters()
{
  // Off-by-1, thruster 1 is thrusters_[0], etc.
  // https://bluerobotics.com/learn/bluerov2-assembly/
#undef JUST_TWO
#ifdef JUST_TWO
// Experiment: use 2 thrusters instead of 4
  thrusters_.emplace_back("t200_link_front_right", false, 2.0, 2.0, 2.0, 0.0);
  thrusters_.emplace_back("t200_link_front_left", false, 2.0, 0.0, 0.0, 0.0);
  thrusters_.emplace_back("t200_link_rear_right", true, 0.0, -2.0, 0.0, 0.0);
  thrusters_.emplace_back("t200_link_rear_left", true, 0.0, 0.0, -2.0, 0.0);
#else
  thrusters_.emplace_back("t200_link_front_right", false, 1.0, 1.0, 1.0, 0.0);
  thrusters_.emplace_back("t200_link_front_left", false, 1.0, -1.0, -1.0, 0.0);
  thrusters_.emplace_back("t200_link_rear_right", true, 1.0, -1.0, 1.0, 0.0);
  thrusters_.emplace_back("t200_link_rear_left", true, 1.0, 1.0, -1.0, 0.0);
#endif
  thrusters_.emplace_back("t200_link_vertical_right", false, 0.0, 0.0, 0.0, 1.0);
  thrusters_.emplace_back("t200_link_vertical_left", true, 0.0, 0.0, 0.0, -1.0);

  // Thrusters and Maestro boot at 1500 (off)
  for (int i = 0; i < 6; ++i) {
    prev_pwm_.push_back(1500);
  }
}

void Thrusters::efforts_to_control(
  const BaseContext & cxt, const mw::Efforts & efforts,
  orca_msgs::msg::Control & control_msg)
{
  // Keep track of saturation for diagnostics
  bool saturated = false;

  control_msg.thruster_pwm.fr_1 = thrusters_[0].efforts_to_pwm(cxt, efforts, saturated);
  control_msg.thruster_pwm.fl_2 = thrusters_[1].efforts_to_pwm(cxt, efforts, saturated);
  control_msg.thruster_pwm.rr_3 = thrusters_[2].efforts_to_pwm(cxt, efforts, saturated);
  control_msg.thruster_pwm.rl_4 = thrusters_[3].efforts_to_pwm(cxt, efforts, saturated);
  control_msg.thruster_pwm.vr_5 = thrusters_[4].efforts_to_pwm(cxt, efforts, saturated);
  control_msg.thruster_pwm.vl_6 = thrusters_[5].efforts_to_pwm(cxt, efforts, saturated);

  control_msg.thruster_saturated = saturated;
}

}  // namespace orca_base
