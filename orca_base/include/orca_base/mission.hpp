#ifndef ORCA_BASE_MISSION_HPP
#define ORCA_BASE_MISSION_HPP

#include "rclcpp_action/rclcpp_action.hpp"

#include "fiducial_vlam_msgs/msg/map.hpp"
#include "orca_msgs/action/mission.hpp"

#include "orca_base/global_planner.hpp"
#include "orca_base/segment.hpp"

namespace orca_base
{

  class Mission
  {
    rclcpp::Logger logger_;                               // ROS logger
    const AUVContext &cxt_;                               // Parameters
    std::shared_ptr<GlobalPlanner> planner_;              // Global planner

    // Mission action state
    std::shared_ptr<rclcpp_action::ServerGoalHandle<orca_msgs::action::Mission>> goal_handle_;
    std::shared_ptr<orca_msgs::action::Mission::Feedback> feedback_;

  public:

    Mission(const rclcpp::Logger &logger, const AUVContext &cxt,
            std::shared_ptr<rclcpp_action::ServerGoalHandle<orca_msgs::action::Mission>> goal_handle,
            std::shared_ptr<GlobalPlanner> planner, const FPStamped &start);

    const PlannerStatus &status() const
    { return planner_->status(); }

    // Advance the plan, return true to continue
    bool advance(rclcpp::Duration d, const FPStamped &estimate, orca::Efforts &efforts);

    // Abort the mission
    void abort();

    // Call the mission a success
    void complete();
  };

} // namespace orca_base

#endif //ORCA_BASE_MISSION_HPP
