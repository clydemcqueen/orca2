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

#ifndef ORCA_SHARED__MW__OBSERVATION_HPP_
#define ORCA_SHARED__MW__OBSERVATION_HPP_

#include "fiducial_vlam_msgs/msg/observation.hpp"
#include "opencv2/core/types.hpp"

namespace mw
{

constexpr int NOT_A_MARKER = -1;

class Observation
{
  int id_{NOT_A_MARKER};
  cv::Point2d c0_;
  cv::Point2d c1_;
  cv::Point2d c2_;
  cv::Point2d c3_;

public:
  static const Observation None;

  Observation() = default;

  explicit Observation(const fiducial_vlam_msgs::msg::Observation & msg)
  {
    id_ = msg.id;
    c0_ = cv::Point2d{msg.x0, msg.y0};
    c1_ = cv::Point2d{msg.x1, msg.y1};
    c2_ = cv::Point2d{msg.x2, msg.y2};
    c3_ = cv::Point2d{msg.x3, msg.y3};
  }

  Observation(
    const int & id,
    const cv::Point2d & c0, const cv::Point2d & c1, const cv::Point2d & c2, const cv::Point2d & c3)
  : id_{id},
    c0_{c0},
    c1_{c1},
    c2_{c2},
    c3_{c3} {}

  fiducial_vlam_msgs::msg::Observation msg() const
  {
    fiducial_vlam_msgs::msg::Observation msg;
    msg.id = id_;
    msg.x0 = c0_.x;
    msg.y0 = c0_.y;
    msg.x1 = c1_.x;
    msg.y1 = c1_.y;
    msg.x2 = c2_.x;
    msg.y2 = c2_.y;
    msg.x3 = c3_.x;
    msg.y3 = c3_.y;
    return msg;
  }

  const int & id() const
  {
    return id_;
  }

  const cv::Point2d & c0() const
  {
    return c0_;
  }

  const cv::Point2d & c1() const
  {
    return c1_;
  }

  const cv::Point2d & c2() const
  {
    return c2_;
  }

  const cv::Point2d & c3() const
  {
    return c3_;
  }

  bool operator==(const Observation & that) const
  {
    return id_ == that.id_ &&
           c0_ == that.c0_ &&
           c1_ == that.c1_ &&
           c2_ == that.c2_ &&
           c3_ == that.c3_;
  }

  bool operator!=(const Observation & that) const
  {
    return !(*this == that);
  }

  friend std::ostream & operator<<(std::ostream & os, const Observation & v);
};

}  // namespace mw

#endif  // ORCA_SHARED__MW__OBSERVATION_HPP_
