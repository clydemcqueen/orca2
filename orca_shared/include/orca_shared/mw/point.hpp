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

#ifndef ORCA_SHARED__MW__POINT_HPP_
#define ORCA_SHARED__MW__POINT_HPP_

#include <cmath>

#include "geometry_msgs/msg/point.hpp"
#include "orca_shared/mw/acceleration.hpp"
#include "orca_shared/mw/twist.hpp"

namespace mw
{

class Point
{
  geometry_msgs::msg::Point msg_;

public:
  Point() = default;

  explicit Point(const geometry_msgs::msg::Point & msg)
  : msg_{msg} {}

  Point(const double & x, const double & y, const double & z)
  {
    msg_.x = x;
    msg_.y = y;
    msg_.z = z;
  }

  geometry_msgs::msg::Point msg() const
  {
    return msg_;
  }

  double x() const
  {
    return msg_.x;
  }

  double y() const
  {
    return msg_.y;
  }

  double z() const
  {
    return msg_.z;
  }

  double & x()
  {
    return msg_.x;
  }

  double & y()
  {
    return msg_.y;
  }

  double & z()
  {
    return msg_.z;
  }

  void x(const double & v)
  {
    msg_.x = v;
  }

  void y(const double & v)
  {
    msg_.y = v;
  }

  void z(const double & v)
  {
    msg_.z = v;
  }

  double distance_xy(const Point & that) const
  {
    return std::hypot(x() - that.x(), y() - that.y());
  }

  double distance_xy(const double & _x, const double & _y) const
  {
    return std::hypot(x() - _x, y() - _y);
  }

  double distance_z(const Point & that) const
  {
    return std::abs(z() - that.z());
  }

  Point motion(const rclcpp::Duration & d, const mw::Twist & v0, const mw::Acceleration & a) const
  {
    auto dt = d.seconds();
    return Point{x() + v0.x() * dt + 0.5 * a.x() * dt * dt,
      y() + v0.y() * dt + 0.5 * a.y() * dt * dt,
      z() + v0.z() * dt + 0.5 * a.z() * dt * dt};
  }

  Point operator+(const Point & that) const
  {
    return Point{x() + that.x(), y() + that.y(), z() + that.z()};
  }

  Point operator-(const Point & that) const
  {
    return Point{x() - that.x(), y() - that.y(), z() - that.z()};
  }

  Point operator-() const
  {
    return Point{-x(), -y(), -z()};
  }

  bool operator==(const Point & that) const
  {
    return msg_ == that.msg_;
  }

  bool operator!=(const Point & that) const
  {
    return !(*this == that);
  }

  friend std::ostream & operator<<(std::ostream & os, const Point & v);
};

}  // namespace mw

#endif  // ORCA_SHARED__MW__POINT_HPP_
