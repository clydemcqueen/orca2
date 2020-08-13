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

#include "orca_base/global_planner.hpp"

#include <memory>
#include <random>
#include <string>
#include <vector>
#include <utility>

#include "orca_msgs/msg/control.hpp"
#include "orca_shared/util.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace orca_base
{

GlobalPlanner::GlobalPlanner(
  const rclcpp::Logger & logger, const AUVContext & cxt, mw::Map map,
  const mw::Observer & observer, const std::string & mission_info, std::vector<mw::Target> targets,
  bool keep_station)
: logger_{logger},
  cxt_{cxt},
  map_{std::move(map)},
  targets_{std::move(targets)},
  keep_station_{keep_station},
  state_{
    observer, mission_info,
    static_cast<int>(targets_.size()),
    static_cast<int>(targets_[0].id())}
{
  RCLCPP_INFO_STREAM(logger_, targets_.size() << " targets:");
  for (auto & i : targets_) {
    RCLCPP_INFO_STREAM(logger_, i);
  }

  // Write global_path_
  global_path_.header.frame_id = cxt_.map_frame_;
  global_path_.poses.clear();
  geometry_msgs::msg::PoseStamped pose_msg;
  for (auto & i : targets_) {
    pose_msg.pose = i.pose().msg();
    global_path_.poses.push_back(pose_msg);
  }
}

void GlobalPlanner::create_pose_planner(const mw::PoseStamped & start)
{
  auto keep_station = state_.target_idx() == targets_.size() - 1 ? keep_station_ : false;
  local_planner_ =
    std::make_shared<PosePlanner>(logger_, cxt_, start, targets_[state_.target_idx()], map_,
      keep_station, state_);
}

void GlobalPlanner::create_mtm_planner(const mw::PolarObservationStamped & start)
{
  local_planner_ = std::make_shared<MoveToMarkerPlanner>(logger_, cxt_, start, state_);
}

// Create a new local planner
bool GlobalPlanner::create_local_planner(const mw::FiducialPoseStamped & estimate)
{
  // If we have a good pose, create a planner
  if (estimate.fp().good(cxt_.good_pose_dist_)) {
    RCLCPP_INFO(logger_, "create local planner from estimated pose");
    create_pose_planner(mw::PoseStamped{estimate.header(), estimate.fp().pose().pose()});
    return true;
  }

  // If we do not expect and do not have a good estimated pose, start from the planned pose
  if (!state_.plan().fp().good(cxt_.good_pose_dist_) && !estimate.fp().good(cxt_.good_pose_dist_)) {
    RCLCPP_INFO(logger_, "create local planner from planned pose");
    create_pose_planner(mw::PoseStamped{state_.plan().header(), state_.plan().fp().pose().pose()});
    return true;
  }

  // Attempt recovery
  if (cxt_.global_plan_allow_mtm_) {
    auto polar = estimate.fp().observations().get_polar(state_.target_marker_id());
    if (polar.distance() < cxt_.good_obs_dist_) {
      RCLCPP_INFO_STREAM(logger_, "recover, move to the target marker " << polar);
      create_mtm_planner(mw::PolarObservationStamped{estimate.header(), polar});
      return true;
    }

    polar = estimate.fp().observations().closest_polar();
    if (polar.distance() < cxt_.good_obs_dist_) {
      RCLCPP_INFO_STREAM(logger_, "recover, move to the closest marker " << polar);
      create_mtm_planner(mw::PolarObservationStamped{estimate.header(), polar});
      return true;
    }
  }

  // Future: spin 360, looking for a marker

  RCLCPP_ERROR(logger_, "can't create a local plan");
  return false;
}

int GlobalPlanner::advance(
  const rclcpp::Duration & d, const mw::FiducialPoseStamped & estimate,
  mw::Efforts & efforts,
  const std::function<void(double, double)> & send_feedback)
{
  // Create a local planner if necessary
  if (!local_planner_ && !create_local_planner(estimate)) {
    return AdvanceRC::FAILURE;
  }

  // Advance the local plan
  if (local_planner_->advance(d, estimate, efforts, state_)) {
    if (local_planner_->is_pose_planner()) {
      // Is there a good pose?
      if (estimate.fp().good(cxt_.good_pose_dist_)) {
        // Good pose, make sure the plan & estimate are reasonably close
        if (estimate.fp().pose().pose().position().distance_xy(
            state_.plan().fp().pose().pose().position()) >
          cxt_.global_plan_max_xy_err_)
        {
          RCLCPP_INFO(logger_, "large pose error, re-plan");
          create_pose_planner(mw::PoseStamped{estimate.header(), estimate.fp().pose().pose()});
          return AdvanceRC::CONTINUE;
        }
      } else if (cxt_.global_plan_allow_mtm_) {
        // Dead reckoning: no marker, or the nearest marker is too far away to calculate a good pose

        // It's possible to detect a marker even if it's too far away to get a good pose
        // We can compare the planned and estimated marker observations

        // First check: if we expect to see the target marker, and we do see the target marker,
        // but the yaw error is high, we can execute a move-to-marker recovery strategy
        // This should keep the marker visible until we're close enough to get a good pose
        auto target_id = targets_[state_.target_idx()].id();
        mw::PolarObservation plan_target = state_.plan().fp().observations().get_polar(target_id);
        mw::PolarObservation estimate_target = estimate.fp().observations().get_polar(target_id);

        if (plan_target.distance() < cxt_.good_obs_dist_ &&
          estimate_target.distance() < cxt_.good_obs_dist_ &&
          std::abs(plan_target.bearing() - estimate_target.bearing()) >
          cxt_.global_plan_max_obs_yaw_err_)
        {
          RCLCPP_INFO(logger_, "poor pose, target marker in view but yaw error is high, recover");
          create_mtm_planner(mw::PolarObservationStamped{estimate.header(), estimate_target});
          return AdvanceRC::CONTINUE;
        }
      }
    }

  } else {  // Local plan is complete
    if (local_planner_->is_pose_planner()) {
      // Move to next target
      if (state_.target_idx() + 1 < targets_.size()) {
        state_.next_target(targets_[state_.target_idx()].id());
        send_feedback(state_.target_idx(), targets_.size());
        RCLCPP_INFO_STREAM(logger_, "next target: " << targets_[state_.target_idx()]);
      } else {
        // Mission complete!
        return AdvanceRC::SUCCESS;
      }
    }

    // Create a new plan next iteration
    local_planner_ = nullptr;
  }

  // If we got this far, continue the mission
  return AdvanceRC::CONTINUE;
}

std::shared_ptr<GlobalPlanner>
GlobalPlanner::plan_markers(
  const rclcpp::Logger & logger, const AUVContext & cxt,
  const mw::Map & map,
  const mw::Observer & observer, const std::string & mission_info,
  const std::vector<int> & markers_ids, bool random,
  bool keep_station)
{
  std::vector<mw::Target> targets{};

  if (markers_ids.empty()) {
    // Use all of the markers in the map
    for (const auto & marker : map.markers()) {
      targets.push_back(
        marker.target(cxt.global_plan_target_z_, cxt.pose_plan_target_dist_, false));
    }
  } else {
    // Use just the markers in the list
    for (const auto marker_id : markers_ids) {
      auto marker = map.get(marker_id);
      if (marker.id() == marker_id) {
        targets.push_back(
          marker.target(cxt.global_plan_target_z_, cxt.pose_plan_target_dist_, false));
      } else {
        RCLCPP_WARN(logger, "marker %d is not in the map, skipping", marker_id);
      }
    }
  }

  if (targets.empty()) {
    RCLCPP_ERROR(logger, "no targets, abort mission");
    return nullptr;
  }

  if (random) {
    // Shuffle targets
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(targets.begin(), targets.end(), g);
  }

  return std::make_shared<GlobalPlanner>(logger, cxt, map, observer, mission_info, targets,
           keep_station);
}

std::shared_ptr<GlobalPlanner>
GlobalPlanner::plan_poses(
  const rclcpp::Logger & logger, const AUVContext & cxt,
  const mw::Map & map,
  const mw::Observer & observer, const std::string & mission_info,
  const std::vector<geometry_msgs::msg::Pose> & poses,
  bool random, bool keep_station)
{
  std::vector<mw::Target> targets{};

  if (poses.empty()) {
    RCLCPP_ERROR(logger, "no poses, abort mission");
    return nullptr;
  }

  for (auto pose : poses) {
    targets.emplace_back(mw::NOT_A_MARKER, mw::Pose{pose});
  }

  if (random) {
    // Shuffle targets
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(targets.begin(), targets.end(), g);
  }

  return std::make_shared<GlobalPlanner>(logger, cxt, map, observer, mission_info, targets,
           keep_station);
}

}  // namespace orca_base
