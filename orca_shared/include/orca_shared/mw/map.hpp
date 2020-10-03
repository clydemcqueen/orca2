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

#ifndef ORCA_SHARED__MW__MAP_HPP_
#define ORCA_SHARED__MW__MAP_HPP_

#include <vector>

#include "fiducial_vlam_msgs/msg/map.hpp"
#include "orca_shared/mw/header.hpp"
#include "orca_shared/mw/marker.hpp"
#include "orca_shared/mw/observations.hpp"
#include "orca_shared/mw/pose.hpp"

namespace mw
{

class Map
{
  fiducial_vlam_msgs::msg::Map msg_;
  std::vector<mw::Marker> markers_;

public:
  Map() = default;

  explicit Map(const fiducial_vlam_msgs::msg::Map & msg)
  : msg_{msg}
  {
    for (int i = 0; i < msg_.ids.size(); ++i) {
      markers_.emplace_back(msg_.ids[i], msg_.marker_length, Pose{msg_.poses[i].pose});
    }
  }

  fiducial_vlam_msgs::msg::Map msg() const
  {
    return msg_;
  }

  bool valid() const
  {
    return Header{msg_.header}.valid();
  }

  double marker_length() const
  {
    return msg_.marker_length;
  }

  const std::vector<mw::Marker> & markers() const
  {
    return markers_;
  }

  const Marker & get(const int & id) const
  {
    for (const auto & item : markers_) {
      if (id == item.id()) {
        return item;
      }
    }
    return Marker::None;
  }

  /**
   * Plan a route from start to destination through the smallest number of waypoints.
   *
   * Ignores marker orientation, so only works for down-facing cameras and markers on the seafloor.
   *
   * @param target_z Travel depth
   * @param max_dead_reckon_dist Max dead reckoning distance
   * @param start_pose Start pose
   * @param destination_pose Destination pose
   * @param waypoints Out: all posts from start to destionat
   * @return True if a path was found
   */
  bool get_waypoints(
    const double & target_z, const double & max_dead_reckon_dist,
    const Pose & start_pose, const Pose & destination_pose, std::vector<Pose> & waypoints) const;

  bool operator==(const Map & that) const
  {
    return msg_ == that.msg_;
  }

  bool operator!=(const Map & that) const
  {
    return !(*this == that);
  }

  friend std::ostream & operator<<(std::ostream & os, const Map & v);
};

}  // namespace mw

#endif  // ORCA_SHARED__MW__MAP_HPP_
