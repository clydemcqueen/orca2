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

#ifndef ORCA_SHARED__MW__POSE_BODY_HPP_
#define ORCA_SHARED__MW__POSE_BODY_HPP_

#include <ostream>
#include <utility>

#include "orca_msgs/msg/pose_body.hpp"

namespace mw
{

class PoseBody
{
  orca_msgs::msg::PoseBody msg_;

public:
  PoseBody() = default;

  explicit PoseBody(const orca_msgs::msg::PoseBody & msg)
    : msg_{msg} {}

  PoseBody(double forward, double strafe, double vertical, double yaw)
  {
    msg_.forward = forward;
    msg_.strafe = strafe;
    msg_.vertical = vertical;
    msg_.yaw = yaw;
  }

  orca_msgs::msg::PoseBody msg() const
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

  double & forward()
  {
    return msg_.forward;
  }

  double & strafe()
  {
    return msg_.strafe;
  }

  double & vertical()
  {
    return msg_.vertical;
  }

  double & yaw()
  {
    return msg_.yaw;
  }

  void forward(const double & v)
  {
    msg_.forward = v;
  }

  void strafe(const double & v)
  {
    msg_.strafe = v;
  }

  void vertical(const double & v)
  {
    msg_.vertical = v;
  }

  void yaw(const double & v)
  {
    msg_.yaw = v;
  }

  PoseBody operator+(const PoseBody & that) const
  {
    return PoseBody{msg_.forward + that.msg_.forward, msg_.strafe + that.msg_.strafe,
      msg_.vertical + that.msg_.vertical, msg_.yaw + that.msg_.yaw};
  }

  PoseBody operator-(const PoseBody & that) const
  {
    return PoseBody{msg_.forward - that.msg_.forward, msg_.strafe - that.msg_.strafe,
      msg_.vertical - that.msg_.vertical, msg_.yaw - that.msg_.yaw};
  }

  PoseBody operator-() const
  {
    return PoseBody{-msg_.forward, -msg_.strafe, -msg_.vertical, -msg_.yaw};
  }

  bool operator==(const PoseBody & that) const
  {
    return msg_.forward == that.msg_.forward &&
      msg_.strafe == that.msg_.strafe &&
      msg_.vertical == that.msg_.vertical &&
      msg_.yaw == that.msg_.yaw;
  }

  bool operator!=(const PoseBody & that) const
  {
    return !(*this == that);
  }

  friend std::ostream & operator<<(std::ostream & os, const PoseBody & v);
};

}  // namespace mw

#endif  // ORCA_SHARED__MW__POSE_BODY_HPP_
