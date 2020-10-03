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

#ifndef ORCA_SHARED__MW__HEADER_HPP_
#define ORCA_SHARED__MW__HEADER_HPP_

#include <string>
#include <utility>

#include "rclcpp/time.hpp"
#include "std_msgs/msg/header.hpp"

namespace mw
{

class Header
{
  rclcpp::Time t_{0L, RCL_ROS_TIME};
  std::string frame_id_;

public:
  Header() = default;

  explicit Header(const std_msgs::msg::Header & msg)
  : t_{msg.stamp, RCL_ROS_TIME},
    frame_id_{msg.frame_id} {}

  Header(const rclcpp::Time & t, std::string frame_id)
  : t_{t},
    frame_id_{std::move(frame_id)} {}

  std_msgs::msg::Header msg() const
  {
    std_msgs::msg::Header msg;
    msg.stamp = t_;
    msg.frame_id = frame_id_;
    return msg;
  }

  bool valid()
  {
    return t().nanoseconds() > 0;
  }

  const rclcpp::Time & t() const
  {
    return t_;
  }

  const std::string & frame_id() const
  {
    return frame_id_;
  }

  rclcpp::Time & t()
  {
    return t_;
  }

  std::string & frame_id()
  {
    return frame_id_;
  }

  bool operator==(const Header & that) const
  {
    return t_ == that.t_ &&
           frame_id_ == that.frame_id_;
  }

  bool operator!=(const Header & that) const
  {
    return !(*this == that);
  }

  friend std::ostream & operator<<(std::ostream & os, const Header & v);
};

}  // namespace mw

#endif  // ORCA_SHARED__MW__HEADER_HPP_
