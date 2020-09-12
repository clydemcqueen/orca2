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

#include "orca_shared/model.hpp"
#include "orca_shared/pwm.hpp"
#include "orca_shared/util.hpp"
#include "rclcpp/logging.hpp"

namespace orca
{

void Model::drag_const_world(
  double yaw, double motion_world, double & drag_const_x,
  double & drag_const_y) const
{
  // Direction of motion in the body frame
  auto motion_body = norm_angle(motion_world - yaw);

  // Fold quadrants II, II and IV into quadrant I
  if (motion_body < 0) {
    motion_body = -motion_body;
  }
  if (motion_body > M_PI / 2) {
    motion_body = M_PI - motion_body;
  }

  // Interpolate between drag_const_s and drag_const_f to find the drag constant
  // for the direction of motion
  auto drag_const_motion =
    (motion_body * drag_const_s() + (M_PI / 2 - motion_body) * drag_const_f()) / (M_PI / 2);

  // Break the drag down to x and y components
  // Coef must be positive, note abs()
  drag_const_x = abs(cos(motion_world)) * drag_const_motion;
  drag_const_y = abs(sin(motion_world)) * drag_const_motion;
}

void Model::log_info(const rclcpp::Logger & logger) const
{
  // Describe hover force, effort and pwm
  auto hover_accel = hover_accel_z();
  auto hover_force = accel_to_force(hover_accel);
  auto hover_effort = force_to_effort_z(hover_force);
  auto hover_pwm = orca::effort_to_pwm(hover_effort);
  RCLCPP_INFO(logger, "hover accel: %g, force: %g, effort: %g, pwm: %d",
    hover_accel, hover_force, hover_effort, hover_pwm);

  // Describe force, effort and pwm for a representative foward velocity
  double fwd_velo = 0.4;
  auto fwd_accel = -drag_accel_f(fwd_velo);
  auto fwd_force = accel_to_force(fwd_accel);
  auto fwd_effort = force_to_effort_xy(fwd_force);
  auto fwd_pwm = orca::effort_to_pwm(fwd_effort);
  RCLCPP_INFO(logger, "fwd velo: %g, accel: %g, force: %g, effort: %g, pwm: %d",
    fwd_velo, fwd_accel, fwd_force, fwd_effort, fwd_pwm);
}

}  // namespace orca
