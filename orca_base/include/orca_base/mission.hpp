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
    std::shared_ptr<BasePlanner> planner_;                // Path planner
    int segment_idx_;                                     // Current segment

    // Mission action state
    std::shared_ptr<rclcpp_action::ServerGoalHandle<orca_msgs::action::Mission>> goal_handle_;
    std::shared_ptr<orca_msgs::action::Mission::Feedback> feedback_;

  public:

    Mission(const rclcpp::Logger &logger, const BaseContext &cxt,
            std::shared_ptr<rclcpp_action::ServerGoalHandle<orca_msgs::action::Mission>> goal_handle,
            std::shared_ptr<BasePlanner> planner,
            const fiducial_vlam_msgs::msg::Map &map, const PoseStamped &start);

    // Advance the plan, return true to continue
    bool advance(double dt, Pose &plan, const nav_msgs::msg::Odometry &estimate, Acceleration &u_bar);

    // Abort the mission
    void abort();
  };

} // namespace orca_base

#endif //ORCA_BASE_MISSION_HPP
