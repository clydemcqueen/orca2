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

#ifndef ORCA_BASE__MOVE_TO_MARKER_PLANNER_HPP_
#define ORCA_BASE__MOVE_TO_MARKER_PLANNER_HPP_

#include <memory>
#include <vector>

#include "rclcpp/logger.hpp"
#include "orca_base/auv_context.hpp"
#include "orca_base/controller.hpp"
#include "orca_base/planner_common.hpp"
#include "orca_base/observation_segment.hpp"

namespace orca_base
{

//=====================================================================================
// MoveToMarkerPlanner -- a recovery strategy
//=====================================================================================

class MoveToMarkerPlanner : public LocalPlanner
{
  rclcpp::Logger logger_;
  const AUVContext & cxt_;

  int marker_id_;                                                   // Target
  std::vector<std::shared_ptr<ObservationSegmentBase>> segments_;   // Motion segments
  std::shared_ptr<ObservationController> controller_;               // Motion controller

public:
  MoveToMarkerPlanner(
    const rclcpp::Logger & logger, const AUVContext & cxt,
    const mw::PolarObservationStamped & start,
    mw::MissionState & state);

  bool advance(
    const rclcpp::Duration & d, const mw::FiducialPoseStamped & estimate,
    mw::Efforts & efforts,
    mw::MissionState & state) override;
};

}  // namespace orca_base

#endif  // ORCA_BASE__MOVE_TO_MARKER_PLANNER_HPP_
