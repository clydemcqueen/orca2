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
    void plan(rclcpp::Logger &logger, const BaseContext &cxt, const fiducial_vlam_msgs::msg::Map &map,
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
    void plan(rclcpp::Logger &logger, const BaseContext &cxt, const fiducial_vlam_msgs::msg::Map &map,
              const PoseStamped &start) override;
  };

  //=====================================================================================
  // ForwardRandomPlanner -- markers are on the walls, and there's a forward-facing camera
  //=====================================================================================

  struct ForwardRandomPlanner : RandomPlanner
  {
    void plan(rclcpp::Logger &logger, const BaseContext &cxt, const fiducial_vlam_msgs::msg::Map &map,
              const PoseStamped &start) override;
  };

  //=====================================================================================
  // Body planners move back/forth along a body axis, for testing motion on that axis
  // X = forward/back, Y = left/right, Z = up/down, Yaw = ccw/cw
  //=====================================================================================

  struct BodyXPlanner : BasePlanner
  {
    void plan(rclcpp::Logger &logger, const BaseContext &cxt, const fiducial_vlam_msgs::msg::Map &map,
              const PoseStamped &start) override;
  };

  struct BodyYPlanner : BasePlanner
  {
    void plan(rclcpp::Logger &logger, const BaseContext &cxt, const fiducial_vlam_msgs::msg::Map &map,
              const PoseStamped &start) override;
  };

  struct BodyZPlanner : BasePlanner
  {
    void plan(rclcpp::Logger &logger, const BaseContext &cxt, const fiducial_vlam_msgs::msg::Map &map,
              const PoseStamped &start) override;
  };

  struct BodyYawPlanner : BasePlanner
  {
    void plan(rclcpp::Logger &logger, const BaseContext &cxt, const fiducial_vlam_msgs::msg::Map &map,
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

  public:

    Mission(rclcpp::Logger logger, std::shared_ptr<BasePlanner> planner, const BaseContext &cxt,
            const fiducial_vlam_msgs::msg::Map &map, const PoseStamped &start);

    // Advance the plan, return true to continue
    bool advance(double dt, Pose &plan, Acceleration &ff);

    const nav_msgs::msg::Path &planned_path() const
    { return planner_->planned_path_; }
  };

} // namespace orca_base

#endif //ORCA_BASE_MISSION_HPP
