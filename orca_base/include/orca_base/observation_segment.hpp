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

#ifndef ORCA_BASE__OBSERVATION_SEGMENT_HPP_
#define ORCA_BASE__OBSERVATION_SEGMENT_HPP_

#include <string>

#include "orca_base/segment_common.hpp"
#include "orca_shared/mw/mw.hpp"

namespace orca_base
{

//=====================================================================================
// Observation segments plan motion based on marker observations
// Observations are in the camera_link frame, not the base_link frame
// This means that the sub is ~20cm further away than obs.distance, and obs.bearing is exaggerated
// In practice we can ignore this
//=====================================================================================

class ObservationSegmentBase : public SegmentBase
{
protected:
  mw::PolarObservationStamped plan_;    // Goal observation
  mw::PolarObservation
    goal_;           // Planned observation, incremented with each call to advance()

  mw::TwistBody twist_;           // Velocity in the body frame
  mw::AccelerationBody ff_;       // Acceleration in the body frame

public:
  ObservationSegmentBase(
    const AUVContext & cxt, uint8_t type, mw::PolarObservationStamped start,
    mw::PolarObservation goal);

  const mw::PolarObservationStamped & plan() const {return plan_;}

  const mw::AccelerationBody & ff() const {return ff_;}
};

//=====================================================================================
// RotateToMarker rotates to face a marker
//=====================================================================================

class RotateToMarker : public ObservationSegmentBase
{
  mw::AccelerationBody initial_accel_;  // Initial total acceleration, not modified
  mw::AccelerationBody accel_;          // Total acceleration, accel_ = drag_ + ff_
  mw::AccelerationBody drag_;           // Acceleration due to drag

  // Start time
  rclcpp::Time start_{0, 0, RCL_ROS_TIME};

  // Time to change from one phase to another
  rclcpp::Time yaw_run_{0, 0, RCL_ROS_TIME};
  rclcpp::Time yaw_decel_{0, 0, RCL_ROS_TIME};
  rclcpp::Time yaw_stop_{0, 0, RCL_ROS_TIME};

public:
  RotateToMarker(
    const AUVContext & cxt, const mw::PolarObservationStamped & start,
    const mw::PolarObservation & goal);

  std::string to_str() override;

  bool advance(const rclcpp::Duration & d) override;
};

//=====================================================================================
// MoveToMarker moves forward toward a marker
//=====================================================================================

class MoveToMarker : public ObservationSegmentBase
{
  mw::AccelerationBody initial_accel_;  // Initial total acceleration, not modified
  mw::AccelerationBody accel_;          // Total acceleration, accel_ = drag_ + ff_
  mw::AccelerationBody drag_;           // Acceleration due to drag

  // Start time
  rclcpp::Time start_{0, 0, RCL_ROS_TIME};

  // Time to change from one phase to another
  rclcpp::Time f_run_{0, 0, RCL_ROS_TIME};
  rclcpp::Time f_decel_{0, 0, RCL_ROS_TIME};
  rclcpp::Time f_stop_{0, 0, RCL_ROS_TIME};

public:
  MoveToMarker(
    const AUVContext & cxt, const mw::PolarObservationStamped & start,
    const mw::PolarObservation & goal);

  std::string to_str() override;

  bool advance(const rclcpp::Duration & d) override;
};

}  // namespace orca_base

#endif  // ORCA_BASE__OBSERVATION_SEGMENT_HPP_
