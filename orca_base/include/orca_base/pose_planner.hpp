#ifndef ORCA_BASE_POSE_PLANNER_HPP
#define ORCA_BASE_POSE_PLANNER_HPP

#include "rclcpp/logger.hpp"

#include "orca_base/auv_context.hpp"
#include "orca_base/controller.hpp"
#include "orca_base/map.hpp"
#include "orca_base/planner_common.hpp"
#include "orca_base/pose_segment.hpp"

namespace orca_base
{

  //=====================================================================================
  // PosePlanner -- given a good pose, build a local plan to a target
  //=====================================================================================

  class PosePlanner : public LocalPlanner
  {
    rclcpp::Logger logger_;
    const AUVContext &cxt_;
    Map map_;

    Target target_;                                             // Target
    bool keep_station_;                                         // True: keep station at target
    std::vector<std::shared_ptr<PoseSegmentBase>> segments_;    // Motion segments
    std::shared_ptr<PoseController> controller_;                // Motion controller
#undef LOCAL_PATH
#ifdef LOCAL_PATH
    nav_msgs::msg::Path local_path_;                            // Path to next target, useful if there are waypoints
#endif

    // Add a trajectory segment and update plan
    void add_keep_station_segment(orca::FPStamped &plan, double seconds);

    void add_vertical_segment(orca::FPStamped &plan, double z);

    void add_rotate_segment(orca::FPStamped &plan, double yaw);

    void add_line_segment(orca::FPStamped &plan, double x, double y);

    void add_pose_segment(orca::FPStamped &plan, const orca::FP &goal);

  public:

    PosePlanner(const rclcpp::Logger &logger, const AUVContext &cxt, const orca::FPStamped &start, Target target,
                Map map, bool keep_station, PlannerStatus &status);

    bool advance(const rclcpp::Duration &d, const orca::FPStamped &estimate, orca::Efforts &efforts,
                 PlannerStatus &status) override;

#ifdef LOCAL_PATH
    const nav_msgs::msg::Path &local_path() const
    { return local_path_; }
#endif
  };

} // namespace orca_base

#endif //ORCA_BASE_POSE_PLANNER_HPP
