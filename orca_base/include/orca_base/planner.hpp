#ifndef ORCA_BASE_PLANNER_HPP
#define ORCA_BASE_PLANNER_HPP

#include "fiducial_vlam_msgs/msg/map.hpp"

#include "orca_base/segment.hpp"

namespace orca_base
{

  //=====================================================================================
  // BasePlanner
  //=====================================================================================

  struct BasePlanner
  {
    std::vector<std::shared_ptr<BaseSegment>> segments_;  // Trajectory segments
    nav_msgs::msg::Path planned_path_;                    // Path for rviz

    virtual void plan(rclcpp::Logger &logger, const BaseContext &cxt, const fiducial_vlam_msgs::msg::Map &map,
                      const PoseStamped &start) = 0;
  };

  //=====================================================================================
  // KeepStationPlanner -- keep current pose
  //=====================================================================================

  struct KeepStationPlanner : BasePlanner
  {
    void plan(rclcpp::Logger &logger, const BaseContext &cxt, const fiducial_vlam_msgs::msg::Map &map,
              const PoseStamped &start) override;
  };

  //=====================================================================================
  // OriginPlanner -- keep station directly below the origin
  //=====================================================================================

  struct OriginPlanner : BasePlanner
  {
    void plan(rclcpp::Logger &logger, const BaseContext &cxt, const fiducial_vlam_msgs::msg::Map &map,
              const PoseStamped &start) override;
  };

  //=====================================================================================
  // RandomPlanner -- base for DownRandomPlanner and ForwardRandomPlanner
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

} // namespace orca_base

#endif //ORCA_BASE_PLANNER_HPP
