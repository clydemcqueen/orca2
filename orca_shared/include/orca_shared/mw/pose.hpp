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

#ifndef ORCA_SHARED__MW__POSE_HPP_
#define ORCA_SHARED__MW__POSE_HPP_

#include <cmath>

#include "geometry_msgs/msg/pose.hpp"
#include "orca_shared/mw/acceleration.hpp"
#include "orca_shared/mw/point.hpp"
#include "orca_shared/mw/quaternion.hpp"
#include "orca_shared/mw/twist.hpp"

namespace mw
{

class Pose
{
  Point position_;
  Quaternion orientation_;

public:
  Pose() = default;

  explicit Pose(const geometry_msgs::msg::Pose & msg)
  : position_{msg.position},
    orientation_{msg.orientation} {}

  Pose(const Point & position, const Quaternion & orientation)
  : position_{position},
    orientation_{orientation} {}

  Pose(const double & x, const double & y, const double & z, const double & yaw)
  : position_{x, y, z},
    orientation_{0, 0, yaw} {}

  geometry_msgs::msg::Pose msg() const
  {
    geometry_msgs::msg::Pose msg;
    msg.position = position_.msg();
    msg.orientation = orientation_.msg();
    return msg;
  }

  const Point & position() const
  {
    return position_;
  }

  const Quaternion & orientation() const
  {
    return orientation_;
  }

  Point & position()
  {
    return position_;
  }

  Quaternion & orientation()
  {
    return orientation_;
  }

  double x() const
  {
    return position_.x();
  }

  double y() const
  {
    return position_.y();
  }

  double z() const
  {
    return position_.z();
  }

  double & x()
  {
    return position_.x();
  }

  double & y()
  {
    return position_.y();
  }

  double & z()
  {
    return position_.z();
  }

  double roll() const
  {
    return orientation_.roll();
  }

  double pitch() const
  {
    return orientation_.pitch();
  }

  double yaw() const
  {
    return orientation_.yaw();
  }

  void roll(const double & v)
  {
    orientation_.roll(v);
  }

  void pitch(const double & v)
  {
    orientation_.pitch(v);
  }

  void yaw(const double & v)
  {
    orientation_.yaw(v);
  }

  // TODO(clyde): return ref to avoid assignment errors, e.g., pose_.transform() = my_transform;
  tf2::Transform transform() const
  {
    tf2::Transform transform;
    tf2::fromMsg(msg(), transform);
    return transform;
  }

  Pose motion(const rclcpp::Duration & d, const mw::Twist & v0, const mw::Acceleration & a) const
  {
    return Pose{position_.motion(d, v0, a), orientation_.motion(d, v0, a)};
  }

  Pose operator+(const Pose & that) const
  {
    return Pose{position_ + that.position_, orientation_ + that.orientation_};
  }

  Pose operator-(const Pose & that) const
  {
    return Pose{position_ - that.position_, orientation_ - that.orientation_};
  }

  Pose operator-() const
  {
    return Pose{-position_, -orientation_};
  }

  bool operator==(const Pose & that) const
  {
    return position_ == that.position_ &&
           orientation_ == that.orientation_;
  }

  bool operator!=(const Pose & that) const
  {
    return !(*this == that);
  }

  friend std::ostream & operator<<(std::ostream & os, const Pose & v);
};

}  // namespace mw

#endif  // ORCA_SHARED__MW__POSE_HPP_
