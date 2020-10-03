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

#ifndef ORCA_BASE__POSE_PLANNER_HPP_
#define ORCA_BASE__POSE_PLANNER_HPP_

#include <memory>
#include <vector>

#include "orca_base/auv_context.hpp"
#include "orca_base/controller.hpp"
#include "orca_base/planner_common.hpp"
#include "orca_base/pose_segment.hpp"
#include "rclcpp/logger.hpp"

namespace orca_base
{

//=====================================================================================
// PosePlanner -- given a good pose, build a local plan to a target
//=====================================================================================

class PosePlanner : public LocalPlanner
{
  rclcpp::Logger logger_;
  const AUVContext & cxt_;
  mw::Map map_;

  mw::Target target_;                                         // Target
  bool keep_station_;                                         // True: keep station at target
  std::vector<std::shared_ptr<PoseSegmentBase>> segments_;    // Motion segments
  std::shared_ptr<PoseController> controller_;                // Motion controller
#undef LOCAL_PATH
#ifdef LOCAL_PATH
  nav_msgs::msg::Path local_path_;  // Path to next target, useful if there are waypoints
#endif

  // Add a trajectory segment and update plan
  void add_pause_segment(mw::PoseStamped & plan, double seconds);

  void add_vertical_segment(mw::PoseStamped & plan, double z);

  void add_rotate_segment(mw::PoseStamped & plan, double yaw);

  void add_line_segment(mw::PoseStamped & plan, double x, double y);

  void add_pose_segment(mw::PoseStamped & plan, const mw::Pose & goal);

public:
  PosePlanner(
    const rclcpp::Logger & logger, const AUVContext & cxt, const mw::PoseStamped & start,
    const mw::Target & target, mw::Map map, bool keep_station, mw::MissionState & state);

  bool advance(
    const rclcpp::Duration & d, const mw::FiducialPoseStamped & estimate,
    mw::Efforts & efforts,
    mw::MissionState & state) override;

#ifdef LOCAL_PATH
  const nav_msgs::msg::Path & local_path() const
  {return local_path_;}
#endif
};

}  // namespace orca_base

#endif  // ORCA_BASE__POSE_PLANNER_HPP_
