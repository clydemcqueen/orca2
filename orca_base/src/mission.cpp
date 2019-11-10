#include "orca_base/mission.hpp"

namespace orca_base
{

  Mission::Mission(const rclcpp::Logger &logger, const BaseContext &cxt,
                   std::shared_ptr<rclcpp_action::ServerGoalHandle<orca_msgs::action::Mission>> goal_handle,
                   std::shared_ptr<BasePlanner> planner,
                   const fiducial_vlam_msgs::msg::Map &map, const PoseStamped &start) :
    logger_{logger},
    cxt_{cxt},
    goal_handle_{std::move(goal_handle)},
    planner_{std::move(planner)}
  {
    // Create path
    planner_->plan(map, start);
    RCLCPP_INFO(logger_, "mission has %d segment(s), segment 0", planner_->segments().size());

    // Init feedback
    if (goal_handle_) {
      feedback_ = std::make_shared<orca_msgs::action::Mission::Feedback>();
      feedback_->segments_completed = 0;
      feedback_->segments_total = planner_->segments().size();
    }

    // Start
    segment_idx_ = 0;
  }

  bool Mission::advance(double dt, Pose &plan, const nav_msgs::msg::Odometry &estimate, Acceleration &u_bar)
  {
    // Cancel this mission?
    if (goal_handle_ && goal_handle_->is_canceling()) {
      RCLCPP_INFO(logger_, "mission cancelled");
      auto result = std::make_shared<orca_msgs::action::Mission::Result>();
      result->segments_total = feedback_->segments_total;
      result->segments_completed = feedback_->segments_completed;
      goal_handle_->canceled(result);
      goal_handle_ = nullptr;
      feedback_ = nullptr;

      // We're done
      return false;
    }

    // Sanity check: if we're in a mission action, make sure we're in a good state
    assert(!goal_handle_ || goal_handle_->is_executing());

    int num_steps = 1;
    constexpr double MAX_STEP = 0.1;

    if (dt > MAX_STEP) {
      // The numerical approximation gets wonky if dt > 0.1. This might happen if the filter times out and restarts.
      // Break a large dt into a number of smaller steps.
      num_steps = std::ceil(dt / MAX_STEP);
      RCLCPP_DEBUG(logger_, "break dt %g into %d steps", dt, num_steps);
      dt /= num_steps;
    }

    Acceleration ff;

    for (int i = 0; i < num_steps; ++i) {

      if (planner_->segments()[segment_idx_]->advance(dt)) {

        // Advance the current motion segment
        plan = planner_->segments()[segment_idx_]->plan();
        ff = planner_->segments()[segment_idx_]->ff();

      } else if (++segment_idx_ < planner_->segments().size()) {

        // The segment is done, move to the next segment
        RCLCPP_INFO(logger_, "mission segment %d / %d", segment_idx_, planner_->segments().size());
        plan = planner_->segments()[segment_idx_]->plan();
        ff = planner_->segments()[segment_idx_]->ff();

        // Send mission feedback
        if (goal_handle_) {
          feedback_->segments_completed = segment_idx_;
          goal_handle_->publish_feedback(feedback_);
        }

      } else {

        // No more segments
        RCLCPP_INFO(logger_, "mission complete");

        // Send mission results
        if (goal_handle_) {
          auto result = std::make_shared<orca_msgs::action::Mission::Result>();
          result->segments_completed = result->segments_total = feedback_->segments_total;
          goal_handle_->succeed(result);
          goal_handle_ = nullptr;
          feedback_ = nullptr;
        }

        // We're done
        return false;
      }
    }

    // Compute acceleration
    planner_->controllers()[segment_idx_]->calc(cxt_, dt, plan, estimate, ff, u_bar);

    // More to do
    return true;
  }

  void Mission::abort()
  {
    RCLCPP_INFO(logger_, "mission aborted");

    if (goal_handle_) {
      auto result = std::make_shared<orca_msgs::action::Mission::Result>();
      result->segments_total = feedback_->segments_total;
      result->segments_completed = feedback_->segments_completed;
      goal_handle_->abort(result);
      goal_handle_ = nullptr;
      feedback_ = nullptr;
    }
  }

} // namespace orca_base
