#ifndef ORCA_BASE_MISSION_HPP
#define ORCA_BASE_MISSION_HPP

#include "fiducial_vlam_msgs/msg/map.hpp"
#include "nav_msgs/msg/path.hpp"

#include "orca_base/base_context.hpp"
#include "orca_base/motion.hpp"

namespace orca_base
{

//=====================================================================================
// BasePlanner
//=====================================================================================

  struct BasePlanner
  {
    std::vector<std::shared_ptr<BaseMotion>> segments_;   // Trajectory segments
    nav_msgs::msg::Path planned_path_;                    // Path for rviz

    virtual void plan(rclcpp::Logger &logger, const BaseContext &cxt, const fiducial_vlam_msgs::msg::Map &map,
                      const PoseStamped &start) = 0;
  };

//=====================================================================================
// KeepStationPlanner
//=====================================================================================

  struct KeepStationPlanner : BasePlanner
  {
    virtual void plan(rclcpp::Logger &logger, const BaseContext &cxt, const fiducial_vlam_msgs::msg::Map &map,
                      const PoseStamped &start) override;
  };

//=====================================================================================
// RandomPlanner
//=====================================================================================

  struct RandomPlanner : BasePlanner
  {
    void plan_from_waypoints(rclcpp::Logger &logger, const BaseContext &cxt, std::vector<Pose> &waypoints,
                             const PoseStamped &start);
  };


//=====================================================================================
// DownRandomPlanner -- markers are on the floor, and there's a down-facing camera
//=====================================================================================

  struct DownRandomPlanner : RandomPlanner
  {
    virtual void plan(rclcpp::Logger &logger, const BaseContext &cxt, const fiducial_vlam_msgs::msg::Map &map,
                      const PoseStamped &start) override;
  };

//=====================================================================================
// ForwardRandomPlanner -- markers are on the walls, and there's a forward-facing camera
//=====================================================================================

  struct ForwardRandomPlanner : RandomPlanner
  {
    virtual void plan(rclcpp::Logger &logger, const BaseContext &cxt, const fiducial_vlam_msgs::msg::Map &map,
                      const PoseStamped &start) override;
  };

//=====================================================================================
// Mission
//=====================================================================================

  class Mission
  {
    rclcpp::Logger logger_;                               // ROS logger
    std::shared_ptr<BasePlanner> planner_;                // Path planner
    int segment_idx_;                                     // Current segment
    PoseError error_;                                     // Total error for this mission

  public:

    Mission(rclcpp::Logger logger, std::shared_ptr<BasePlanner> planner, const BaseContext &cxt,
            const fiducial_vlam_msgs::msg::Map &map, const PoseStamped &start);

    // Advance the controller, return true to continue
    bool advance(const double dt, const PoseStamped &curr, Acceleration &u_bar);

    const nav_msgs::msg::Path &planned_path() const
    { return planner_->planned_path_; }

    const PoseError &error() const
    { return error_; }
  };

} // namespace orca_base

#endif //ORCA_BASE_MISSION_HPP
