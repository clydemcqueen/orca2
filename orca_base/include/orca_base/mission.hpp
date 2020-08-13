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

#ifndef ORCA_BASE__MISSION_HPP_
#define ORCA_BASE__MISSION_HPP_

#include <memory>

#include "orca_base/global_planner.hpp"
#include "orca_msgs/action/mission.hpp"
#include "orca_shared/mw/efforts.hpp"
#include "orca_shared/mw/fiducial_pose_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace orca_base
{

class Mission
{
  rclcpp::Logger logger_;                               // ROS logger
  const AUVContext & cxt_;                               // Parameters
  std::shared_ptr<GlobalPlanner> planner_;              // Global planner

  // Mission action state
  std::shared_ptr<rclcpp_action::ServerGoalHandle<orca_msgs::action::Mission>> goal_handle_;
  std::shared_ptr<orca_msgs::action::Mission::Feedback> feedback_;

public:
  Mission(
    const rclcpp::Logger & logger, const AUVContext & cxt,
    std::shared_ptr<rclcpp_action::ServerGoalHandle<orca_msgs::action::Mission>> goal_handle,
    std::shared_ptr<GlobalPlanner> planner, const mw::FiducialPoseStamped & start);

  const mw::MissionState & status() const {return planner_->status();}

  // Advance the plan, return true to continue
  bool advance(
    const rclcpp::Duration & d, const mw::FiducialPoseStamped & estimate,
    mw::Efforts & efforts);

  // Abort the mission
  void abort();

  // Call the mission a success
  void complete();
};

}  // namespace orca_base

#endif  // ORCA_BASE__MISSION_HPP_
