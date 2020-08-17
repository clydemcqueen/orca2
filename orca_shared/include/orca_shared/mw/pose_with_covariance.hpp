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

#ifndef ORCA_SHARED__MW__POSE_WITH_COVARIANCE_HPP_
#define ORCA_SHARED__MW__POSE_WITH_COVARIANCE_HPP_

#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "orca_shared/mw/pose.hpp"

namespace mw
{

using CovarianceType = std::array<double, 36>;

class PoseWithCovariance
{
  Pose pose_;
  CovarianceType covariance_{};

public:
  constexpr static int X_IX = 0 * 7;
  constexpr static int Y_IX = 1 * 7;
  constexpr static int Z_IX = 2 * 7;
  constexpr static int ROLL_IX = 3 * 7;
  constexpr static int PITCH_IX = 4 * 7;
  constexpr static int YAW_IX = 5 * 7;

  constexpr static double THRESHOLD = 1e4;
  constexpr static double VERY_HIGH = 1e5;

  constexpr static int DOF_NONE = 0;
  constexpr static int DOF_Z = 1;
  constexpr static int DOF_FOUR = 2;
  constexpr static int DOF_SIX = 3;

  PoseWithCovariance() = default;

  explicit PoseWithCovariance(const geometry_msgs::msg::PoseWithCovariance & msg)
  : pose_{msg.pose},
    covariance_{msg.covariance} {}

  PoseWithCovariance(const Pose & pose, const CovarianceType & covariance, int dof = DOF_SIX)
  : pose_{pose},
    covariance_{covariance}
  {
    // Slam the covariance as required
    if (dof != DOF_SIX) {
      covariance_[ROLL_IX] = VERY_HIGH;
      covariance_[PITCH_IX] = VERY_HIGH;
    }
    if (dof != DOF_SIX && dof != DOF_FOUR) {
      covariance_[X_IX] = VERY_HIGH;
      covariance_[Y_IX] = VERY_HIGH;
      covariance_[YAW_IX] = VERY_HIGH;
    }
    if (dof != DOF_SIX && dof != DOF_FOUR && dof != DOF_Z) {
      covariance_[Z_IX] = VERY_HIGH;
    }
  }

  geometry_msgs::msg::PoseWithCovariance msg() const
  {
    geometry_msgs::msg::PoseWithCovariance msg;
    msg.pose = pose_.msg();
    msg.covariance = covariance_;
    return msg;
  }

  const Pose & pose() const
  {
    return pose_;
  }

  const std::array<double, 36> & covariance() const
  {
    return covariance_;
  }

  Pose & pose()
  {
    return pose_;
  }

  std::array<double, 36> & covariance()
  {
    return covariance_;
  }

  int dof() const
  {
    if (covariance_[Z_IX] > THRESHOLD) {
      return DOF_NONE;
    } else if (covariance_[X_IX] > THRESHOLD || covariance_[Y_IX] > THRESHOLD ||
      covariance_[YAW_IX] > THRESHOLD)
    {
      return DOF_Z;
    } else if (covariance_[ROLL_IX] > THRESHOLD || covariance_[PITCH_IX] > THRESHOLD) {
      return DOF_FOUR;
    } else {
      return DOF_SIX;
    }
  }

  bool good1() const
  {
    auto d = dof();
    return d == DOF_SIX || d == DOF_FOUR || d == DOF_Z;
  }

  bool good4() const
  {
    auto d = dof();
    return d == DOF_SIX || d == DOF_FOUR;
  }

  bool good6() const
  {
    return dof() == DOF_SIX;
  }

  bool operator==(const PoseWithCovariance & that) const
  {
    return pose_ == that.pose_ &&
           covariance_ == that.covariance_;
  }

  bool operator!=(const PoseWithCovariance & that) const
  {
    return !(*this == that);
  }

  friend std::ostream & operator<<(std::ostream & os, const PoseWithCovariance & v);
};

}  // namespace mw

#endif  // ORCA_SHARED__MW__POSE_WITH_COVARIANCE_HPP_
