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

#ifndef ORCA_BASE__GLOBAL_PLANNER_HPP_
#define ORCA_BASE__GLOBAL_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav_msgs/msg/path.hpp"
#include "orca_base/pose_planner.hpp"
#include "orca_base/move_to_marker_planner.hpp"
#include "orca_description/parser.hpp"

namespace orca_base
{

//=====================================================================================
// GlobalPlanner -- orchestrate planning to reach a list of targets
//=====================================================================================

class GlobalPlanner
{
  rclcpp::Logger logger_;
  const AUVContext & cxt_;
  mw::Map map_;

  // Plan
  std::vector<mw::Target> targets_;                         // Global plan
  bool keep_station_;                                       // True: keep station at last target
  nav_msgs::msg::Path global_path_;                         // Path to all targets
  std::shared_ptr<LocalPlanner> local_planner_;             // Local planner
  mw::MissionState state_;                                  // Mission state

  /**
   * Create a pose planner
   * @param start Initial pose
   */
  void create_pose_planner(const mw::PoseStamped & start);

  /**
   * Create a move-to-marker planner
   * @param start
   */
  void create_mtm_planner(const mw::PolarObservationStamped & start);

  /**
   * Create some sort of local planner if possible
   * @param estimate Current pose
   * @return True if successful
   */
  bool create_local_planner(const mw::FiducialPoseStamped & estimate);

public:
  explicit GlobalPlanner(
    const rclcpp::Logger & logger, const AUVContext & cxt, mw::Map map,
    const mw::Observer & observer, const std::string & mission_info,
    std::vector<mw::Target> targets, bool keep_station);

  const mw::MissionState & status() const {return state_;}

  const nav_msgs::msg::Path & global_path() const {return global_path_;}

  // Advance the plan by dt, return AdvanceRC
  int advance(
    const rclcpp::Duration & d, const mw::FiducialPoseStamped & estimate,
    mw::Efforts & efforts,
    const std::function<void(double completed, double total)> & send_feedback);

  // Factory: visit a list markers, if list is empty all markers will be visited
  static std::shared_ptr<GlobalPlanner>
  plan_markers(
    const rclcpp::Logger & logger, const AUVContext & cxt, const mw::Map & map,
    const mw::Observer & observer, const std::string & mission_info,
    const std::vector<int> & markers_ids, bool random,
    bool keep_station);

  // Factory: visit a list of poses, list cannot be empty
  static std::shared_ptr<GlobalPlanner>
  plan_poses(
    const rclcpp::Logger & logger, const AUVContext & cxt, const mw::Map & map,
    const mw::Observer & observer, const std::string & mission_info,
    const std::vector<geometry_msgs::msg::Pose> & poses, bool random,
    bool keep_station);
};

}  // namespace orca_base

#endif  // ORCA_BASE__GLOBAL_PLANNER_HPP_
