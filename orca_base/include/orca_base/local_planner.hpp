#ifndef ORCA_BASE_LOCAL_PLANNER_HPP
#define ORCA_BASE_LOCAL_PLANNER_HPP

#include "rclcpp/logger.hpp"

#include "orca_base/auv_context.hpp"
#include "orca_base/controller.hpp"
#include "orca_base/map.hpp"
#include "orca_base/planner_common.hpp"
#include "orca_base/segment.hpp"

namespace orca_base
{

  //=====================================================================================
  // LocalPlanner -- build a local plan to a target
  //=====================================================================================

  class LocalPlanner
  {
    rclcpp::Logger logger_;
    const AUVContext &cxt_;
    Map map_;

    Target target_;                                             // Target
    bool keep_station_;                                         // True: keep station at target
    std::vector<std::shared_ptr<PoseSegmentBase>> segments_;    // Motion segments
    std::shared_ptr<PoseController> controller_;                // Motion controller
    nav_msgs::msg::Path local_path_;                            // Path to next target

    // Add a trajectory segment and update plan
    void add_keep_station_segment(FPStamped &plan, double seconds);

    void add_vertical_segment(FPStamped &plan, double z);

    void add_rotate_segment(FPStamped &plan, double yaw);

    void add_line_segment(FPStamped &plan, double x, double y);

    void add_pose_segment(FPStamped &plan, const FP &goal);

  public:

    LocalPlanner(const rclcpp::Logger &logger, const AUVContext &cxt, const FPStamped &start, Target target, Map map,
                 bool keep_station, PlannerStatus &status);

    bool advance(const rclcpp::Duration &d, const FPStamped &estimate, orca::Efforts &efforts, PlannerStatus &status);

    // TODO publish local path
    const nav_msgs::msg::Path &local_path() const
    { return local_path_; }
  };

} // namespace orca_base

#endif //ORCA_BASE_LOCAL_PLANNER_HPP
