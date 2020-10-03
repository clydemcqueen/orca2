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

#ifndef ORCA_SHARED__MW__OBSERVATIONS_HPP_
#define ORCA_SHARED__MW__OBSERVATIONS_HPP_

#include <limits>
#include <vector>

#include "fiducial_vlam_msgs/msg/observations.hpp"
#include "orca_msgs/msg/observations.hpp"
#include "orca_shared/mw/observer.hpp"
#include "orca_shared/mw/observation.hpp"
#include "orca_shared/mw/polar_observation.hpp"
#include "orca_shared/mw/pose.hpp"

namespace mw
{

class Observations
{
  Observer observer_;
  std::vector<Observation> observations_;
  std::vector<PolarObservation> polar_observations_;

public:
  Observations() = default;

  explicit Observations(const orca_msgs::msg::Observations & msg)
  : observer_{msg.observer}
  {
    assert(msg.observations.size() == msg.polar_observations.size());
    for (const auto & item : msg.observations) {
      observations_.emplace_back(item);
    }
    for (const auto & item : msg.polar_observations) {
      polar_observations_.emplace_back(item);
    }
  }

  // From fiducial_vlam_msgs + geometry_msgs
  Observations(
    const double & marker_length,
    const geometry_msgs::msg::Pose & cam_f_base,
    const fiducial_vlam_msgs::msg::Observations & vlam_observations)
  : observer_{marker_length, vlam_observations.camera_info, cam_f_base}
  {
    for (const auto & item : vlam_observations.observations) {
      add(item);
    }
  }

  explicit Observations(const Observer & observer)
  : observer_{observer} {}

  orca_msgs::msg::Observations msg() const
  {
    orca_msgs::msg::Observations msg;
    msg.observer = observer_.msg();
    for (const auto & item : observations_) {
      msg.observations.emplace_back(item.msg());
    }
    for (const auto & item : polar_observations_) {
      msg.polar_observations.emplace_back(item.msg());
    }
    return msg;
  }

  const Observer & observer() const
  {
    return observer_;
  }

  const std::vector<Observation> & observations() const
  {
    return observations_;
  }

  const std::vector<PolarObservation> & polar_observations() const
  {
    return polar_observations_;
  }

  Observer & observer()
  {
    return observer_;
  }

  bool size() const
  {
    return observations().size();
  }

  bool empty() const
  {
    return observations_.empty();
  }

  void clear()
  {
    observations_.clear();
    polar_observations_.clear();
  }

  const Observation & get(const int & id) const
  {
    for (const auto & item : observations_) {
      if (id == item.id()) {
        return item;
      }
    }
    return Observation::None;
  }

  const PolarObservation & get_polar(const int & id) const
  {
    for (const auto & item : polar_observations_) {
      if (id == item.id()) {
        return item;
      }
    }
    return PolarObservation::None;
  }

  const PolarObservation & closest_polar() const
  {
    // 2 passes required to return a const &PolarObservation
    auto closest_distance = std::numeric_limits<double>::max();
    auto closest_id = NOT_A_MARKER;

    // Pass 1
    for (const auto & item : polar_observations_) {
      if (item.distance() < closest_distance) {
        closest_distance = item.distance();
        closest_id = item.id();
      }
    }

    // Pass 2
    return get_polar(closest_id);
  }

  double closest_distance() const
  {
    auto closest = closest_polar();
    return closest.distance();
  }

  bool good(const double & max_distance) const
  {
    return closest_distance() < max_distance;
  }

  void add(const Observation & observation)
  {
    PolarObservation polar_observation;
    observer_.convert(observation, polar_observation);

    observations_.push_back(observation);
    polar_observations_.push_back(polar_observation);
  }

  void add(const fiducial_vlam_msgs::msg::Observation & vlam_observation)
  {
    add(Observation{vlam_observation});
  }

  void add_polar(const PolarObservation & polar_observation)
  {
    Observation observation;
    observer_.convert(polar_observation, observation);

    observations_.push_back(observation);
    polar_observations_.push_back(polar_observation);
  }

  bool operator==(const Observations & that) const
  {
    if (observations_.size() != that.observations_.size()) {
      return false;
    }
    for (int i = 0; i < observations_.size(); ++i) {
      if (observations_[i] != that.observations_[i]) {
        return false;
      }
    }

    return true;
  }

  bool operator!=(const Observations & that) const
  {
    return !(*this == that);
  }

  friend std::ostream & operator<<(std::ostream & os, const Observations & v);
};

}  // namespace mw

#endif  // ORCA_SHARED__MW__OBSERVATIONS_HPP_
