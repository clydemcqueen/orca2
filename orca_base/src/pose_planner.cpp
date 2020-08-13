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

#include "orca_base/pose_planner.hpp"

#include <memory>
#include <utility>
#include <vector>

#include "rclcpp/logging.hpp"
#include "orca_msgs/msg/control.hpp"

namespace orca_base
{

//=====================================================================================
// PosePlanner -- build a local plan to a target
//=====================================================================================

PosePlanner::PosePlanner(
  const rclcpp::Logger & logger, const AUVContext & cxt,
  const mw::PoseStamped & start,
  const mw::Target & target, mw::Map map, bool keep_station, mw::MissionState & state)
: LocalPlanner{LocalPlannerType::POSE_PLANNER},
  logger_{logger},
  cxt_{cxt},
  target_{target},
  map_{std::move(map)},
  keep_station_{keep_station},
  controller_{std::make_shared<PoseController>(cxt_)}
{
  // Generate a plan to move from start to target. Use parameters to control the details.

  // Start pose
  mw::PoseStamped plan = start;

  // ament_cmake_cpplint and ament_cmake_uncrustify don't agree on what to do with
  // long "else if" tests. Hack around this for now.
  auto t1xy = plan.pose().position().distance_xy(target_.pose().position()) <
    cxt_.pose_plan_epsilon_xyz_;
  auto t1z = plan.pose().position().distance_z(target_.pose().position()) <
    cxt_.pose_plan_epsilon_xyz_;
  auto t1yaw = plan.pose().orientation().distance_yaw(target_.pose().orientation()) <
    cxt_.pose_plan_epsilon_yaw_;
  auto t2 = target.pose().position().distance_xy(start.pose().position()) <
    cxt_.pose_plan_max_short_plan_xy_;

  if (t1xy && t1z && t1yaw) {
    // Case 1: no motion plan, just use the PID controllers
    // Set the epsilons high for PID tuning, but otherwise should be very small
    RCLCPP_INFO_STREAM(logger_, "no plan " << target);
    plan.pose() = target_.pose();
  } else if (t2) {
    // Case 2: "small" amount of xy motion, use a single segment to move from start to target
    RCLCPP_INFO_STREAM(logger_, "short plan " << target);
    add_pose_segment(plan, target.pose());
  } else {
    // Case 3: long distance motion
    // Find a series of waypoints (if possible), and always face the direction of motion
    RCLCPP_INFO_STREAM(logger_, "long plan " << target);

    // Generate a series of waypoints to minimize dead reckoning
    std::vector<mw::Pose> waypoints;

    if (cxt_.pose_plan_waypoints_ &&
      map_.get_waypoints(cxt_.global_plan_target_z_, cxt_.pose_plan_max_dead_reckon_dist_,
      start.pose(), target_.pose(), waypoints))
    {
      RCLCPP_INFO(logger_, "... through %d waypoints", waypoints.size() - 1);
    } else {
      waypoints.push_back(target_.pose());
    }

    // Travel to each waypoint, breaking down z, yaw and xy phases
    for (auto & waypoint : waypoints) {
      // Ascend/descend to target z
      if (plan.pose().position().distance_z(waypoint.position()) > cxt_.pose_plan_epsilon_xyz_) {
        add_vertical_segment(plan, waypoint.z());
        add_pause_segment(plan, cxt_.pose_plan_pause_duration_);
      } else {
        RCLCPP_INFO(logger_, "skip vertical");
      }

      if (plan.pose().position().distance_xy(waypoint.position()) > cxt_.pose_plan_epsilon_xyz_) {
        // Point in the direction of travel
        auto direction_of_travel =
          atan2(waypoint.y() - plan.pose().y(), waypoint.x() - plan.pose().x());
        if (plan.pose().orientation().distance_yaw(direction_of_travel) >
          cxt_.pose_plan_epsilon_yaw_)
        {
          add_rotate_segment(plan, direction_of_travel);
        } else {
          RCLCPP_INFO(logger_, "skip rotate");
        }
        add_pause_segment(plan, cxt_.pose_plan_pause_duration_);

        // Travel
        add_line_segment(plan, waypoint.x(), waypoint.y());
      } else {
        RCLCPP_INFO(logger_, "skip travel");
      }
    }

    // Rotate to the target yaw
    if (plan.pose().orientation().distance_yaw(target_.pose().orientation()) >
      cxt_.pose_plan_epsilon_yaw_)
    {
      add_rotate_segment(plan, target_.pose().yaw());
    } else {
      RCLCPP_INFO(logger_, "skip final rotate");
    }
  }

  // Pause
  add_pause_segment(plan, cxt_.pose_plan_pause_duration_);

#ifdef LOCAL_PATH
  // Create a path message to this target for diagnostics
  local_path_.header.frame_id = cxt_.map_frame_;
  local_path_.poses.clear();

  geometry_msgs::msg::PoseStamped pose_msg;
  for (auto & i : segments_) {
    pose_msg.pose = i->plan().fp.pose.pose.to_msg();
    local_path_.poses.push_back(pose_msg);
  }

  // Add last goal pose to path message
  pose_msg.pose = segments_.back()->goal().pose.pose.to_msg();
  local_path_.poses.push_back(pose_msg);
#endif

  if (keep_station_) {
    // Keep station at the last target
    add_pause_segment(plan, 1e6);
  }

  RCLCPP_INFO(logger_, "planned duration %g seconds",
    (plan.header().t() - start.header().t()).seconds());

  // Update state
  state.first_segment(orca_msgs::msg::MissionState::PLAN_LOCAL, segments_.size(),
    segments_[0]->to_str(), segments_[0]->type());
  RCLCPP_INFO_STREAM(logger_, "segment 1 of " << segments_.size() << ", " << state.segment_info());

  state.set_plan(segments_[state.segment_idx()]->plan(), map_);
  state.twist() = segments_[state.segment_idx()]->twist();
}

void PosePlanner::add_pause_segment(mw::PoseStamped & plan, double seconds)
{
  if (seconds > 0) {
    segments_.push_back(
      std::make_shared<Pause>(cxt_, plan, rclcpp::Duration::from_seconds(seconds)));
  }
}

void PosePlanner::add_vertical_segment(mw::PoseStamped & plan, double z)
{
  segments_.push_back(Trap2::make_vertical(cxt_, plan, z));
}

void PosePlanner::add_rotate_segment(mw::PoseStamped & plan, double yaw)
{
  segments_.push_back(Trap2::make_rotate(cxt_, plan, yaw));
}

void PosePlanner::add_line_segment(mw::PoseStamped & plan, double x, double y)
{
  segments_.push_back(Trap2::make_line(cxt_, plan, x, y));
}

void PosePlanner::add_pose_segment(mw::PoseStamped & plan, const mw::Pose & goal)
{
  segments_.push_back(Trap2::make_pose(cxt_, plan, goal));
}

bool PosePlanner::advance(
  const rclcpp::Duration & d, const mw::FiducialPoseStamped & estimate,
  mw::Efforts & efforts,
  mw::MissionState & state)
{
  // Update the plan
  if (segments_[state.segment_idx()]->advance(d)) {
    // Continue in this segment
  } else if (state.segment_idx() + 1 < segments_.size()) {
    // Move to the next segment
    state.next_segment(segments_[state.segment_idx() + 1]->to_str(),
      segments_[state.segment_idx() + 1]->type());
    RCLCPP_INFO_STREAM(logger_,
      "segment " << state.segment_idx() + 1 << " of " << segments_.size() << ", " <<
      state.segment_info());
  } else {
    // Local plan is complete
    return false;
  }

  state.set_plan(segments_[state.segment_idx()]->plan(), map_);
  state.twist() = segments_[state.segment_idx()]->twist();
  state.phase() = segments_[state.segment_idx()]->phase();

  // Run PID controller and calculate efforts
  controller_->calc(d, state.plan().fp(), estimate.fp(), segments_[state.segment_idx()]->ff(),
    efforts);

  return true;
}

}  // namespace orca_base
