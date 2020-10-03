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

#ifndef ORCA_SHARED__MW__MARKER_HPP_
#define ORCA_SHARED__MW__MARKER_HPP_

#include "image_geometry/pinhole_camera_model.h"
#include "orca_shared/mw/observation.hpp"
#include "orca_shared/mw/target.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Vector3.h"

namespace mw
{

class Marker
{
  int id_{};
  double marker_length_{};
  Pose pose_;
  tf2::Vector3 corner0_f_map_;
  tf2::Vector3 corner1_f_map_;
  tf2::Vector3 corner2_f_map_;
  tf2::Vector3 corner3_f_map_;

public:
  static const Marker None;

  Marker() = default;

  Marker(const int & id, const double & marker_length, const Pose & pose)
  : id_{id},
    marker_length_{marker_length},
    pose_{pose}
  {
    tf2::Transform t_map_marker = pose.transform();
    corner0_f_map_ = t_map_marker * tf2::Vector3{-marker_length / 2.f, marker_length / 2.f, 0.f};
    corner1_f_map_ = t_map_marker * tf2::Vector3{marker_length / 2.f, marker_length / 2.f, 0.f};
    corner2_f_map_ = t_map_marker * tf2::Vector3{marker_length / 2.f, -marker_length / 2.f, 0.f};
    corner3_f_map_ = t_map_marker * tf2::Vector3{-marker_length / 2.f, -marker_length / 2.f, 0.f};
  }

  int id() const
  {
    return id_;
  }

  double marker_length() const
  {
    return marker_length_;
  }

  Observation predict_observation(
    const image_geometry::PinholeCameraModel & cam_model,
    const tf2::Transform & t_cam_map) const;

  // Floor == true: markers must be on the floor facing up, and there must be a down-facing camera
  // Floor == false: markers must be on the wall, and there must be a forward-facing camera
  // TODO(clyde): add support for rotated markers, e.g., the sideways markers used in ft3
  Target target(const double & target_z, const double & target_dist, const bool & floor) const
  {
    Pose target_pose = pose_;

    // Set plan.z from parameters
    target_pose.z() = target_z;

    if (!floor) {
      // Target is in front of the marker
      target_pose.x() += sin(target_pose.yaw()) * target_dist;
      target_pose.y() -= cos(target_pose.yaw()) * target_dist;

      // Face the marker to get a good pose
      target_pose.yaw(target_pose.yaw() + M_PI_2);
    }

    return {id(), target_pose};
  }

  bool operator==(const Marker & that) const
  {
    return id_ == that.id_ &&
           marker_length_ == that.marker_length_ &&
           pose_ == that.pose_;
  }

  bool operator!=(const Marker & that) const
  {
    return !(*this == that);
  }

  friend std::ostream & operator<<(std::ostream & os, const Marker & v);
};

}  // namespace mw

#endif  // ORCA_SHARED__MW__MARKER_HPP_
