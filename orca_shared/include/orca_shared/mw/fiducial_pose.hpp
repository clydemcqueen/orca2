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

#ifndef ORCA_SHARED__MW__FIDUCIAL_POSE_HPP_
#define ORCA_SHARED__MW__FIDUCIAL_POSE_HPP_

#include "geometry_msgs/msg/pose.hpp"
#include "orca_msgs/msg/fiducial_pose.hpp"
#include "orca_shared/mw/map.hpp"
#include "orca_shared/mw/observations.hpp"
#include "orca_shared/mw/pose_with_covariance.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace mw
{

class FiducialPose
{
  Observations observations_;
  PoseWithCovariance pose_;

public:
  FiducialPose() = default;

  explicit FiducialPose(const orca_msgs::msg::FiducialPose & msg)
  : observations_{msg.observations},
    pose_{msg.pose} {}

  // From fiducial_vlam_msgs + geometry_msgs
  FiducialPose(
    const double & marker_length,
    const geometry_msgs::msg::Pose & cam_f_base,
    const fiducial_vlam_msgs::msg::Observations & vlam_observations,
    const geometry_msgs::msg::PoseWithCovariance & pose)
  : observations_{marker_length, cam_f_base, vlam_observations},
    pose_{pose} {}

  FiducialPose(const Observations & observations, const PoseWithCovariance & pose)
  : observations_{observations},
    pose_{pose} {}

  explicit FiducialPose(const Observer & observer)
  : observations_{observer} {}

  orca_msgs::msg::FiducialPose msg() const
  {
    orca_msgs::msg::FiducialPose msg;
    msg.observations = observations_.msg();
    msg.pose = pose_.msg();
    return msg;
  }

  const Observations & observations() const
  {
    return observations_;
  }

  const PoseWithCovariance & pose() const
  {
    return pose_;
  }

  Observations & observations()
  {
    return observations_;
  }

  PoseWithCovariance & pose()
  {
    return pose_;
  }

  bool good(const double & max_distance) const
  {
    return pose().good4() && observations().closest_distance() < max_distance;
  }

  /**
   * Clear observations, and predict observations from a map
   * @param map Map of markers
   * @return Number of predicted observations
   */
  int predict_observations(const Map & map);

  bool operator==(const FiducialPose & that) const
  {
    return observations_ == that.observations_ &&
           pose_ == that.pose_;
  }

  bool operator!=(const FiducialPose & that) const
  {
    return !(*this == that);
  }

  friend std::ostream & operator<<(std::ostream & os, const FiducialPose & v);
};
}  // namespace mw

#endif  // ORCA_SHARED__MW__FIDUCIAL_POSE_HPP_
