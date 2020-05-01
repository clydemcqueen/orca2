#ifndef ORCA_BASE_GLOBAL_PLANNER_HPP
#define ORCA_BASE_GLOBAL_PLANNER_HPP

#include "nav_msgs/msg/path.hpp"
#include "orca_base/pose_planner.hpp"
#include "orca_base/move_to_marker_planner.hpp"
#include "orca_description/parser.hpp"

namespace orca_base
{

  //=====================================================================================
  // GlobalPlanner -- orchestrate planning to reach a list of targets
  //=====================================================================================

  class GlobalPlanner
  {
    rclcpp::Logger logger_;
    const AUVContext &cxt_;
    mw::Map map_;

    // Plan
    std::vector<mw::Target> targets_;                         // Global plan
    bool keep_station_;                                       // True: keep station at last target
    nav_msgs::msg::Path global_path_;                         // Path to all targets
    std::shared_ptr<LocalPlanner> local_planner_;             // Local planner
    mw::MissionState state_;                                  // Mission state

    /**
     * Create a pose planner
     * @param start Initial pose
     */
    void create_pose_planner(const mw::PoseStamped &start);

    /**
     * Create a move-to-marker planner
     * @param start
     */
    void create_mtm_planner(const mw::PolarObservationStamped &start);

    /**
     * Create some sort of local planner if possible
     * @param estimate Current pose
     * @return True if successful
     */
    bool create_local_planner(const mw::FiducialPoseStamped &estimate);

  public:

    explicit GlobalPlanner(const rclcpp::Logger &logger, const AUVContext &cxt, mw::Map map,
                           const mw::Observer &observer, std::vector<mw::Target> targets, bool keep_station);

    const mw::MissionState &status() const
    { return state_; }

    const nav_msgs::msg::Path &global_path() const
    { return global_path_; }

    // Advance the plan by dt, return AdvanceRC
    int advance(const rclcpp::Duration &d, const mw::FiducialPoseStamped &estimate, mw::Efforts &efforts,
                const std::function<void(double completed, double total)> &send_feedback);

    // Factory: visit a list markers, if list is empty all markers will be visited
    static std::shared_ptr<GlobalPlanner>
    plan_markers(const rclcpp::Logger &logger, const AUVContext &cxt, const mw::Map &map,
                 const mw::Observer &observer, const std::vector<int> &markers_ids, bool random, bool repeat,
                 bool keep_station);

    // Factory: visit a list of poses, list cannot be empty
    static std::shared_ptr<GlobalPlanner>
    plan_poses(const rclcpp::Logger &logger, const AUVContext &cxt, const mw::Map &map,
               const mw::Observer &observer, const std::vector<geometry_msgs::msg::Pose> &poses, bool random,
               bool repeat, bool keep_station);
  };

} // namespace orca_base

#endif //ORCA_BASE_GLOBAL_PLANNER_HPP
