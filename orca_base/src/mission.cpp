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

#include "orca_base/mission.hpp"

#include <memory>
#include <utility>

namespace orca_base
{
Mission::Mission(
  const rclcpp::Logger & logger, const AUVContext & cxt,
  std::shared_ptr<rclcpp_action::ServerGoalHandle<orca_msgs::action::Mission>> goal_handle,
  std::shared_ptr<GlobalPlanner> planner, const mw::FiducialPoseStamped & start)
: logger_{logger},
  cxt_{cxt},
  goal_handle_{std::move(goal_handle)},
  planner_{std::move(planner)}
{
  // Init feedback
  if (goal_handle_) {
    feedback_ = std::make_shared<orca_msgs::action::Mission::Feedback>();
    feedback_->targets_completed = 0;
    feedback_->targets_total = 0;
  }
}

bool Mission::advance(
  const rclcpp::Duration & d, const mw::FiducialPoseStamped & estimate,
  mw::Efforts & efforts)
{
  // Cancel this mission?
  if (goal_handle_ && goal_handle_->is_canceling()) {
    RCLCPP_INFO(logger_, "mission cancelled");
    auto result = std::make_shared<orca_msgs::action::Mission::Result>();
    result->targets_total = feedback_->targets_total;
    result->targets_completed = feedback_->targets_completed;
    goal_handle_->canceled(result);
    goal_handle_ = nullptr;
    feedback_ = nullptr;

    // We're done
    return false;
  }

  // Sanity check: if we're in a mission action, make sure we're in a good state
  assert(!goal_handle_ || goal_handle_->is_executing());

  auto send_feedback = [&](int completed, int total)
    {
      if (goal_handle_) {
        feedback_->targets_completed = completed;
        feedback_->targets_total = total;
        goal_handle_->publish_feedback(feedback_);
      }
    };

  // Advance the global plan
  auto rc = planner_->advance(d, estimate, efforts, send_feedback);
  if (rc == AdvanceRC::FAILURE) {
    abort();
    return false;
  } else if (rc == AdvanceRC::SUCCESS) {
    complete();
    return false;
  }

  // The mission continues
  return true;
}

void Mission::abort()
{
  RCLCPP_INFO(logger_, "mission aborted");

  if (goal_handle_) {
    auto result = std::make_shared<orca_msgs::action::Mission::Result>();
    result->targets_total = feedback_->targets_total;
    result->targets_completed = feedback_->targets_completed;
    goal_handle_->abort(result);
    goal_handle_ = nullptr;
    feedback_ = nullptr;
  }
}

void Mission::complete()
{
  RCLCPP_INFO(logger_, "mission succeeded");

  if (goal_handle_) {
    auto result = std::make_shared<orca_msgs::action::Mission::Result>();
    result->targets_total = result->targets_completed = feedback_->targets_total;
    goal_handle_->succeed(result);
    goal_handle_ = nullptr;
    feedback_ = nullptr;
  }
}

}  // namespace orca_base
