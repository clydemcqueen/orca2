#ifndef ORCA_BASE_GLOBAL_PLANNER_HPP
#define ORCA_BASE_GLOBAL_PLANNER_HPP

#include "orca_description/parser.hpp"

#include "orca_base/local_planner.hpp"
#include "orca_base/move_to_marker_planner.hpp"

namespace orca_base
{

  //=====================================================================================
  // GlobalPlanner -- orchestrate planning to reach a list of targets
  //=====================================================================================

  class GlobalPlanner
  {
    rclcpp::Logger logger_;
    const BaseContext &cxt_;
    Map map_;
    orca_description::Parser parser_;
    image_geometry::PinholeCameraModel fcam_model_;

    // Plan
    std::vector<Target> targets_;                             // Global plan
    bool keep_station_;                                       // True: keep station at last target
    nav_msgs::msg::Path global_path_;                         // Path to all targets
    std::shared_ptr<LocalPlanner> local_planner_;             // Local planner, or...
    std::shared_ptr<MoveToMarkerPlanner> recovery_planner_;   // ... recovery planner

    // State
    PlannerStatus status_;                                    // Planner status

    // Given a planned pose, predict the marker observations for the forward camera
    void predict_observations(FP &plan) const;

    // Create a local_planner_
    void start_local_plan(const FPStamped &start);

    // Create a recovery_planner_
    void start_recovery_plan(const ObservationStamped &start);

  public:

    explicit GlobalPlanner(const rclcpp::Logger &logger, const BaseContext &cxt, Map map,
                           orca_description::Parser parser,
                           const image_geometry::PinholeCameraModel &fcam_model, std::vector<Target> targets,
                           bool keep_station);

    const PlannerStatus &status() const
    { return status_; }

    const nav_msgs::msg::Path &global_path() const
    { return global_path_; }

    // Advance the plan by dt, return AdvanceRC
    int advance(const rclcpp::Duration &d, const FPStamped &estimate, orca::Efforts &efforts,
                const std::function<void(double completed, double total)> &send_feedback);

    // Factory: visit a list markers, if list is empty all markers will be visited
    static std::shared_ptr<GlobalPlanner>
    plan_markers(const rclcpp::Logger &logger, const BaseContext &cxt, const Map &map,
                 const orca_description::Parser &parser,
                 const image_geometry::PinholeCameraModel &fcam_model,
                 const std::vector<int> &markers_ids, bool random, bool repeat, bool keep_station);

    // Factory: visit a list of poses, list cannot be empty
    static std::shared_ptr<GlobalPlanner>
    plan_poses(const rclcpp::Logger &logger, const BaseContext &cxt, const Map &map,
               const orca_description::Parser &parser, const image_geometry::PinholeCameraModel &fcam_model,
               const std::vector<geometry_msgs::msg::Pose> &poses, bool random, bool repeat, bool keep_station);
  };

} // namespace orca_base

#endif //ORCA_BASE_GLOBAL_PLANNER_HPP
