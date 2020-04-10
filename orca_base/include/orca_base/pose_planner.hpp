#ifndef ORCA_BASE_POSE_PLANNER_HPP
#define ORCA_BASE_POSE_PLANNER_HPP

#include "orca_base/auv_context.hpp"
#include "orca_base/controller.hpp"
#include "orca_base/planner_common.hpp"
#include "orca_base/pose_segment.hpp"
#include "rclcpp/logger.hpp"

namespace orca_base
{

  //=====================================================================================
  // PosePlanner -- given a good pose, build a local plan to a target
  //=====================================================================================

  class PosePlanner : public LocalPlanner
  {
    rclcpp::Logger logger_;
    const AUVContext &cxt_;
    mw::Map map_;

    mw::Target target_;                                         // Target
    bool keep_station_;                                         // True: keep station at target
    std::vector<std::shared_ptr<PoseSegmentBase>> segments_;    // Motion segments
    std::shared_ptr<PoseController> controller_;                // Motion controller
#undef LOCAL_PATH
#ifdef LOCAL_PATH
    nav_msgs::msg::Path local_path_;                            // Path to next target, useful if there are waypoints
#endif

    // Add a trajectory segment and update plan
    void add_keep_station_segment(mw::PoseStamped &plan, double seconds);

    void add_vertical_segment(mw::PoseStamped &plan, double z);

    void add_rotate_segment(mw::PoseStamped &plan, double yaw);

    void add_line_segment(mw::PoseStamped &plan, double x, double y);

    void add_pose_segment(mw::PoseStamped &plan, const mw::Pose &goal);

  public:

    PosePlanner(const rclcpp::Logger &logger, const AUVContext &cxt, const mw::PoseStamped &start, mw::Target target,
                mw::Map map, bool keep_station, mw::MissionState &state);

    bool advance(const rclcpp::Duration &d, const mw::FiducialPoseStamped &estimate, mw::Efforts &efforts,
                 mw::MissionState &state) override;

#ifdef LOCAL_PATH
    const nav_msgs::msg::Path &local_path() const
    { return local_path_; }
#endif
  };

} // namespace orca_base

#endif //ORCA_BASE_POSE_PLANNER_HPP
