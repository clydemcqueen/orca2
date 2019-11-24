#ifndef ORCA_BASE_MISSION_HPP
#define ORCA_BASE_MISSION_HPP

#include "rclcpp_action/rclcpp_action.hpp"

#include "fiducial_vlam_msgs/msg/map.hpp"
#include "orca_msgs/action/mission.hpp"

#include "orca_base/planner.hpp"
#include "orca_base/segment.hpp"

namespace orca_base
{

  class Mission
  {
    rclcpp::Logger logger_;                               // ROS logger
    const BaseContext &cxt_;                              // Parameters
    std::shared_ptr<PlannerBase> planner_;                // Path planner

    // Mission action state
    std::shared_ptr<rclcpp_action::ServerGoalHandle<orca_msgs::action::Mission>> goal_handle_;
    std::shared_ptr<orca_msgs::action::Mission::Feedback> feedback_;

  public:

    Mission(const rclcpp::Logger &logger, const BaseContext &cxt,
            std::shared_ptr<rclcpp_action::ServerGoalHandle<orca_msgs::action::Mission>> goal_handle,
            std::shared_ptr<PlannerBase> planner, const orca::PoseStamped &start);

    const nav_msgs::msg::Path &planned_path() const
    { return planner_->planned_path(); }

    // Advance the plan, return true to continue
    bool advance(double dt, orca::Pose &plan, const nav_msgs::msg::Odometry &estimate, orca::Acceleration &u_bar);

    // Abort the mission
    void abort();

    // Call the mission a success
    void complete();
  };

} // namespace orca_base

#endif //ORCA_BASE_MISSION_HPP
