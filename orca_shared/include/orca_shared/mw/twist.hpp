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

#ifndef ORCA_SHARED__MW__TWIST_HPP_
#define ORCA_SHARED__MW__TWIST_HPP_

#include "geometry_msgs/msg/twist.hpp"

namespace mw
{

class Twist
{
  geometry_msgs::msg::Twist msg_;

public:
  Twist() = default;

  explicit Twist(const geometry_msgs::msg::Twist & msg)
  : msg_{msg} {}

  Twist(const double & x, const double & y, const double & z, const double & yaw)
  {
    msg_.linear.x = x;
    msg_.linear.y = y;
    msg_.linear.z = z;
    msg_.angular.z = yaw;
  }

  geometry_msgs::msg::Twist msg() const
  {
    return msg_;
  }

  double x() const
  {
    return msg_.linear.x;
  }

  double y() const
  {
    return msg_.linear.y;
  }

  double z() const
  {
    return msg_.linear.z;
  }

  double yaw() const
  {
    return msg_.angular.z;
  }

  double & x()
  {
    return msg_.linear.x;
  }

  double & y()
  {
    return msg_.linear.y;
  }

  double & z()
  {
    return msg_.linear.z;
  }

  double & yaw()
  {
    return msg_.angular.z;
  }

  void x(const double & v)
  {
    msg_.linear.x = v;
  }

  void y(const double & v)
  {
    msg_.linear.y = v;
  }

  void z(const double & v)
  {
    msg_.linear.z = v;
  }

  void yaw(const double & v)
  {
    msg_.angular.z = v;
  }

  Twist motion(const rclcpp::Duration & d, const mw::Acceleration & a) const
  {
    double dt = d.seconds();
    return Twist{x() + a.x() * dt,
      y() + a.y() * dt,
      z() + a.z() * dt,
      yaw() + a.yaw() * dt};
  }

  bool operator==(const Twist & that) const
  {
    return msg_ == that.msg_;
  }

  bool operator!=(const Twist & that) const
  {
    return !(*this == that);
  }

  friend std::ostream & operator<<(std::ostream & os, const Twist & v);
};

}  // namespace mw

#endif  // ORCA_SHARED__MW__TWIST_HPP_
