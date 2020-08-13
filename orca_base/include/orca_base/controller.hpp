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

#ifndef ORCA_BASE__CONTROLLER_HPP_
#define ORCA_BASE__CONTROLLER_HPP_

#include "orca_base/auv_context.hpp"
#include "orca_base/pid.hpp"
#include "orca_shared/mw/mw.hpp"

namespace orca_base
{

//=====================================================================================
// PoseController uses 4 pid controllers in the world frame to compute efforts
//=====================================================================================

class PoseController
{
  const AUVContext & cxt_;

  // PID controllers
  pid::Controller x_controller_;
  pid::Controller y_controller_;
  pid::Controller z_controller_;
  pid::Controller yaw_controller_;

public:
  explicit PoseController(const AUVContext & cxt);

  void
  calc(
    const rclcpp::Duration & d, const mw::FiducialPose & plan, const mw::FiducialPose & estimate,
    const mw::Acceleration & ff,
    mw::Efforts & efforts);
};

//=====================================================================================
// Observation controller uses fiducial_vlam observations, not poses
// Observations are in the body frame, not the world frame
//=====================================================================================

class ObservationController
{
  const AUVContext & cxt_;

  // PID controllers
  pid::Controller forward_controller_;
  pid::Controller vertical_controller_;
  pid::Controller bearing_controller_;

public:
  explicit ObservationController(const AUVContext & cxt);

  // The observation is pretty noisy for z, so pass in z data from the barometer
  void
  calc(
    const rclcpp::Duration & d, const mw::PolarObservation & plan, double plan_z,
    const mw::PolarObservation & estimate,
    double estimate_z, const mw::AccelerationBody & ff, mw::Efforts & efforts);
};

}  // namespace orca_base

#endif  // ORCA_BASE__CONTROLLER_HPP_
