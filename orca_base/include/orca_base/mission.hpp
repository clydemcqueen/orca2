#ifndef ORCA_BASE_MISSION_HPP
#define ORCA_BASE_MISSION_HPP

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
    const AUVContext &cxt_;                               // Parameters
    std::shared_ptr<GlobalPlanner> planner_;              // Global planner

    // Mission action state
    std::shared_ptr<rclcpp_action::ServerGoalHandle<orca_msgs::action::Mission>> goal_handle_;
    std::shared_ptr<orca_msgs::action::Mission::Feedback> feedback_;

  public:

    Mission(const rclcpp::Logger &logger, const AUVContext &cxt,
            std::shared_ptr<rclcpp_action::ServerGoalHandle<orca_msgs::action::Mission>> goal_handle,
            std::shared_ptr<GlobalPlanner> planner, const mw::FiducialPoseStamped &start);

    const mw::MissionState &status() const
    { return planner_->status(); }

    // Advance the plan, return true to continue
    bool advance(const rclcpp::Duration& d, const mw::FiducialPoseStamped &estimate, mw::Efforts &efforts);

    // Abort the mission
    void abort();

    // Call the mission a success
    void complete();
  };

} // namespace orca_base

#endif //ORCA_BASE_MISSION_HPP
