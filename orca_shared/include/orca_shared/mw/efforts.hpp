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

#ifndef ORCA_SHARED__MW__EFFORTS_HPP_
#define ORCA_SHARED__MW__EFFORTS_HPP_

#include <utility>

#include "orca_msgs/msg/efforts.hpp"
#include "orca_shared/model.hpp"
#include "orca_shared/mw/acceleration.hpp"
#include "orca_shared/util.hpp"

namespace mw
{

class Efforts
{
  orca_msgs::msg::Efforts msg_;

public:
  Efforts() = default;

  explicit Efforts(const orca_msgs::msg::Efforts & msg)
  {
    msg_.forward = msg.forward;
    msg_.strafe = msg.strafe;
    msg_.vertical = msg.vertical;
    msg_.yaw = msg.yaw;
  }

  Efforts(const orca::Model & model, const double current_yaw, const Acceleration & u_bar)
  {
    // Convert from world frame to body frame
    double x_effort = model.accel_to_effort_xy(u_bar.x());
    double y_effort = model.accel_to_effort_xy(u_bar.y());
    double forward_effort, strafe_effort;
    orca::rotate_frame(x_effort, y_effort, current_yaw, forward_effort, strafe_effort);

    forward(forward_effort);
    strafe(strafe_effort);
    vertical(model.accel_to_effort_z(u_bar.z()));
    yaw(model.accel_to_effort_yaw(u_bar.yaw()));
  }

  orca_msgs::msg::Efforts to_msg() const
  {
    return msg_;
  }

  double forward() const
  {
    return msg_.forward;
  }

  double strafe() const
  {
    return msg_.strafe;
  }

  double vertical() const
  {
    return msg_.vertical;
  }

  double yaw() const
  {
    return msg_.yaw;
  }

  void forward(const double & v)
  {
    msg_.forward = orca::clamp(v, -1.0, 1.0);
  }

  void strafe(const double & v)
  {
    msg_.strafe = orca::clamp(v, -1.0, 1.0);
  }

  void vertical(const double & v)
  {
    msg_.vertical = orca::clamp(v, -1.0, 1.0);
  }

  // TODO(clyde): rename 'rotate'
  void yaw(const double & v)
  {
    msg_.yaw = orca::clamp(v, -1.0, 1.0);
  }

  void all_stop()
  {
    msg_.forward = 0;
    msg_.strafe = 0;
    msg_.vertical = 0;
    msg_.yaw = 0;
  }

  Acceleration acceleration(const orca::Model & model, const double current_yaw) const
  {
    // Acceleration to effort
    double forward_accel = model.effort_to_accel_xy(msg_.forward);
    double strafe_accel = model.effort_to_accel_xy(msg_.strafe);
    double z_accel = model.effort_to_accel_z(msg_.vertical);
    double yaw_accel = model.effort_to_accel_yaw(msg_.yaw);

    // Rotate from body frame to world frame
    double x_accel, y_accel;
    orca::rotate_frame(forward_accel, strafe_accel, -current_yaw, x_accel, y_accel);

    return Acceleration{x_accel, y_accel, z_accel, yaw_accel};
  }
};

std::ostream & operator<<(std::ostream & os, Efforts const & e);

}  // namespace mw

#endif  // ORCA_SHARED__MW__EFFORTS_HPP_
