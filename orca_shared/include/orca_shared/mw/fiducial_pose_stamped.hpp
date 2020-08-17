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

#ifndef ORCA_SHARED__MW__FIDUCIAL_POSE_STAMPED_HPP_
#define ORCA_SHARED__MW__FIDUCIAL_POSE_STAMPED_HPP_

#include <nav_msgs/msg/path__struct.hpp>
#include "orca_msgs/msg/fiducial_pose_stamped.hpp"
#include "orca_shared/mw/fiducial_pose.hpp"
#include "orca_shared/mw/header.hpp"
#include "orca_shared/mw/pose_stamped.hpp"

namespace mw
{

class FiducialPoseStamped
{
  Header header_;
  FiducialPose fp_;

public:
  FiducialPoseStamped() = default;

  explicit FiducialPoseStamped(const orca_msgs::msg::FiducialPoseStamped & msg)
  : header_{msg.header},
    fp_{msg.fp} {}

  // From fiducial_vlam_msgs + geometry_msgs
  FiducialPoseStamped(
    const double & marker_length,
    const geometry_msgs::msg::Pose & cam_f_base,
    const fiducial_vlam_msgs::msg::Observations & vlam_observations,
    const geometry_msgs::msg::PoseWithCovariance & pose)
  : header_{vlam_observations.header},
    fp_{marker_length, cam_f_base, vlam_observations, pose} {}

  FiducialPoseStamped(const Header & header, const FiducialPose & fp)
  : header_{header},
    fp_{fp} {}

  explicit FiducialPoseStamped(const Observer & observer)
  : fp_{observer} {}

  orca_msgs::msg::FiducialPoseStamped msg() const
  {
    orca_msgs::msg::FiducialPoseStamped msg;
    msg.header = header_.msg();
    msg.fp = fp_.msg();
    return msg;
  }

  const Header & header() const
  {
    return header_;
  }

  const FiducialPose & fp() const
  {
    return fp_;
  }

  Header & header()
  {
    return header_;
  }

  FiducialPose & fp()
  {
    return fp_;
  }

  void add_to_path(nav_msgs::msg::Path & path)
  {
    path.poses.push_back(PoseStamped{header(), fp().pose().pose()}.msg());
  }

  bool operator==(const FiducialPoseStamped & that) const
  {
    return header_ == that.header_ &&
           fp_ == that.fp_;
  }

  bool operator!=(const FiducialPoseStamped & that) const
  {
    return !(*this == that);
  }

  friend std::ostream & operator<<(std::ostream & os, const FiducialPoseStamped & v);
};

}  // namespace mw

#endif  // ORCA_SHARED__MW__FIDUCIAL_POSE_STAMPED_HPP_
