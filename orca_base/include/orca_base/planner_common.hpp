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

#ifndef ORCA_BASE__PLANNER_COMMON_HPP_
#define ORCA_BASE__PLANNER_COMMON_HPP_

#include <orca_shared/mw/mw.hpp>
#include "orca_shared/mw/efforts.hpp"
#include "orca_shared/mw/mission_state.hpp"

namespace orca_base
{

//=====================================================================================
// AdvanceRC
//=====================================================================================

struct AdvanceRC
{
  static constexpr int CONTINUE = 0;
  static constexpr int SUCCESS = 1;
  static constexpr int FAILURE = 2;
};

//=====================================================================================
// LocalPlannerType
//=====================================================================================

struct LocalPlannerType
{
  static constexpr int POSE_PLANNER = 0;
  static constexpr int MTM_PLANNER = 1;
  // Future: 360 planner
};

//=====================================================================================
// LocalPlanner -- abstract base class
//=====================================================================================

class LocalPlanner
{
  int type_;

public:
  explicit LocalPlanner(int type)
  : type_{type} {}

  bool is_pose_planner() {return type_ == LocalPlannerType::POSE_PLANNER;}

  virtual bool advance(
    const rclcpp::Duration & d, const mw::FiducialPoseStamped & estimate,
    mw::Efforts & efforts,
    mw::MissionState & status) = 0;
};

}  // namespace orca_base

#endif  // ORCA_BASE__PLANNER_COMMON_HPP_
