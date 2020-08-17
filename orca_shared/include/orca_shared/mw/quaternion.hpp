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

#ifndef ORCA_SHARED__MW__QUATERNION_HPP_
#define ORCA_SHARED__MW__QUATERNION_HPP_

#include <cmath>

#include "geometry_msgs/msg/quaternion.hpp"
#include "orca_shared/util.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace mw
{

class Quaternion
{
  double roll_{};
  double pitch_{};
  double yaw_{};

public:
  Quaternion() = default;

  explicit Quaternion(const geometry_msgs::msg::Quaternion & msg)
  {
    tf2::Quaternion q;
    tf2::fromMsg(msg, q);
    tf2::Matrix3x3(q).getRPY(roll_, pitch_, yaw_);
  }

  Quaternion(const double & roll, const double & pitch, const double & yaw)
  : roll_{roll},
    pitch_{pitch},
    yaw_{yaw} {}

  geometry_msgs::msg::Quaternion msg() const
  {
    tf2::Matrix3x3 m;
    m.setRPY(roll_, pitch_, yaw_);
    tf2::Quaternion q;
    m.getRotation(q);
    return tf2::toMsg(q);
  }

  double roll() const
  {
    return roll_;
  }

  double pitch() const
  {
    return pitch_;
  }

  double yaw() const
  {
    return yaw_;
  }

  void roll(const double & v)
  {
    roll_ = orca::norm_angle(v);
  }

  void pitch(const double & v)
  {
    pitch_ = orca::norm_angle(v);
  }

  void yaw(const double & v)
  {
    yaw_ = orca::norm_angle(v);
  }

  double distance_yaw(const Quaternion & that) const
  {
    return std::abs(orca::norm_angle(yaw() - that.yaw()));
  }

  double distance_yaw(const double & that) const
  {
    return std::abs(orca::norm_angle(yaw() - that));
  }

  Quaternion
  motion(const rclcpp::Duration & d, const mw::Twist & v0, const mw::Acceleration & a) const
  {
    auto dt = d.seconds();
    return Quaternion{roll_, pitch_,
      orca::norm_angle(yaw_ + v0.yaw() * dt + 0.5 * a.yaw() * dt * dt)};
  }

  Quaternion operator+(const Quaternion & that) const
  {
    return Quaternion{roll_ + that.roll_, pitch_ + that.pitch_, yaw_ + that.yaw_};
  }

  Quaternion operator-(const Quaternion & that) const
  {
    return Quaternion{roll_ - that.roll_, pitch_ - that.pitch_, yaw_ - that.yaw_};
  }

  Quaternion operator-() const
  {
    return Quaternion{-roll_, -pitch_, -yaw_};
  }

  bool operator==(const Quaternion & that) const
  {
    return roll_ == that.roll_ &&
           pitch_ == that.pitch_ &&
           yaw_ == that.yaw_;
  }

  bool operator!=(const Quaternion & that) const
  {
    return !(*this == that);
  }

  friend std::ostream & operator<<(std::ostream & os, const Quaternion & v);
};

}  // namespace mw

#endif  // ORCA_SHARED__MW__QUATERNION_HPP_
