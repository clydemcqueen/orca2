#include "orca_base/mission.hpp"

namespace orca_base
{

  Mission::Mission(const rclcpp::Logger &logger, const BaseContext &cxt,
                   std::shared_ptr<rclcpp_action::ServerGoalHandle<orca_msgs::action::Mission>> goal_handle,
                   std::shared_ptr<PlannerBase> planner, const PoseStamped &start) :
    logger_{logger},
    cxt_{cxt},
    goal_handle_{std::move(goal_handle)},
    planner_{std::move(planner)}
  {
    // Create path
    RCLCPP_INFO(logger_, "mission has %d targets(s), target 1", planner_->targets().size());

    // Init feedback
    if (goal_handle_) {
      feedback_ = std::make_shared<orca_msgs::action::Mission::Feedback>();
      feedback_->targets_completed = 0;
      feedback_->targets_total = 0;
    }
  }

  bool Mission::advance(double dt, Pose &plan, const nav_msgs::msg::Odometry &estimate, Acceleration &u_bar)
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

      auto send_feedback = [&](int completed, int total)
      {
        if (goal_handle_) {
          feedback_->targets_completed = completed;
          feedback_->targets_total = total;
          goal_handle_->publish_feedback(feedback_);
        }
      };

      auto rc = planner_->advance(dt, plan, estimate, u_bar, send_feedback);
      if (rc == AdvanceRC::FAILURE) {
        abort();
        return false;
      } else if (rc == AdvanceRC::SUCCESS) {
        complete();
        return false;
      }
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
    RCLCPP_INFO(logger_, "mission completed");

    if (goal_handle_) {
      auto result = std::make_shared<orca_msgs::action::Mission::Result>();
      result->targets_total = result->targets_completed = feedback_->targets_total;
      goal_handle_->succeed(result);
      goal_handle_ = nullptr;
      feedback_ = nullptr;
    }
  }

} // namespace orca_base
