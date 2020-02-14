#ifndef ORCA_BASE_PLANNER_HPP
#define ORCA_BASE_PLANNER_HPP

#include "image_geometry/pinhole_camera_model.h"

#include "orca_description/parser.hpp"
#include "orca_shared/geometry.hpp"

#include "orca_base/map.hpp"
#include "orca_base/segment.hpp"

namespace orca_base
{

  struct AdvanceRC
  {
    static constexpr int CONTINUE = 0;
    static constexpr int SUCCESS = 1;
    static constexpr int FAILURE = 2;
  };

  //=====================================================================================
  // Target
  //=====================================================================================

  struct Target
  {
    int marker_id;
    orca::FP fp;

    Target() : marker_id{orca::NOT_A_MARKER}
    {}

    Target(int _marker_id, orca::FP _fp) : marker_id{_marker_id}, fp{std::move(_fp)}
    {}
  };

  std::ostream &operator<<(std::ostream &os, Target const &target);

  //=====================================================================================
  // LocalPlanner -- build a local plan to a target
  //=====================================================================================

  class LocalPlanner
  {
    rclcpp::Logger logger_;
    const BaseContext &cxt_;
    Map map_;

    Target target_;                                             // Target
    bool keep_station_;                                         // True: keep station at target
    std::vector<std::shared_ptr<PoseSegmentBase>> segments_;    // Motion segments
    int segment_idx_;                                           // Current segment
    std::shared_ptr<PoseController> controller_;                // Motion controller
    nav_msgs::msg::Path local_path_;                            // Path to next target

    // Add a trajectory segment and update plan
    void add_keep_station_segment(orca::FPStamped &plan, double seconds);

    void add_vertical_segment(orca::FPStamped &plan, double z);

    void add_rotate_segment(orca::FPStamped &plan, double yaw);

    void add_line_segment(orca::FPStamped &plan, double x, double y);

    void add_pose_segment(orca::FPStamped &plan, const orca::FP &goal);

  public:

    LocalPlanner(const rclcpp::Logger &logger, const BaseContext &cxt, const orca::FPStamped &start, Target target,
      Map map, bool keep_station);

    bool advance(const rclcpp::Duration &d, orca::FPStamped &plan, const orca::FPStamped &estimate, orca::Efforts &efforts,
                const std::function<void(double completed, double total)> &send_feedback);

    const Target &target() const
    { return target_; }

    const nav_msgs::msg::Path &local_path() const
    { return local_path_; }
  };

  //=====================================================================================
  // MoveToMarkerPlanner -- a recovery strategy
  //=====================================================================================

  class MoveToMarkerPlanner
  {
    rclcpp::Logger logger_;
    const BaseContext &cxt_;

    int marker_id_;                                                   // Target
    std::vector<std::shared_ptr<ObservationSegmentBase>> segments_;   // Motion segments
    int segment_idx_;                                                 // Current segment
    std::shared_ptr<MoveToMarkerController> controller_;              // Motion controller

  public:

    MoveToMarkerPlanner(const rclcpp::Logger &logger, const BaseContext &cxt, const orca::Observation &start);

    bool advance(const rclcpp::Duration &d, orca::FPStamped &plan, const orca::FPStamped &estimate, orca::Efforts &efforts,
                const std::function<void(double completed, double total)> &send_feedback);

    int marker_id() const
    { return marker_id_; }
  };

  //=====================================================================================
  // MissionPlanner -- orchestrate planning to reach a list of targets
  //=====================================================================================

  class MissionPlanner
  {
    rclcpp::Logger logger_;
    const BaseContext &cxt_;
    Map map_;
    orca_description::Parser parser_;
    image_geometry::PinholeCameraModel fcam_model_;

    // Plan & plan state
    std::vector<Target> targets_;                             // Global plan
    bool keep_station_;                                       // True: keep station at last target
    int target_idx_;                                          // Current target
    nav_msgs::msg::Path global_path_;                         // Path to all targets
    std::shared_ptr<LocalPlanner> local_planner_;             // Local planner, or...
    std::shared_ptr<MoveToMarkerPlanner> recovery_planner_;   // ... recovery planner

    // Given a planned pose, predict the marker observations for the forward camera
    void predict_observations(orca::FP &plan) const;

    // Finalize the global plan, called by plan_*()
    void finish_global_plan();

    // Create a local_planner_
    void start_local_plan(const orca::FPStamped &start);

    // Create a recovery_planner_
    void start_recovery_plan(const orca::Observation &start);

  public:

    MissionPlanner(const rclcpp::Logger &logger, const BaseContext &cxt, Map map, orca_description::Parser parser,
                   const image_geometry::PinholeCameraModel &fcam_model_);

    const std::vector<Target> &targets() const
    { return targets_; }

    const nav_msgs::msg::Path &global_path() const
    { return global_path_; }

    int target_marker_id() const
    { return targets_.empty() ? orca::NOT_A_MARKER : targets_[target_idx_].marker_id; }

    bool in_recovery() const
    { return recovery_planner_ != nullptr; }

    int recovery_marker_id() const
    { return recovery_planner_ == nullptr ? orca::NOT_A_MARKER : recovery_planner_->marker_id(); }

    // Global planner: move to a pose, optionally keeping station when we get there
    void plan_target(const orca::FP &fp, bool keep_station);

    // Global planner: visit all markers on the floor (for down-facing camera(s))
    void plan_floor_markers(bool random);

    // Global planner: visit all markers on the wall (for a forward-facing camera)
    void plan_wall_markers(bool random);

    // Advance the plan by dt, return AdvanceRC
    int advance(const rclcpp::Duration &d, orca::FPStamped &plan, const orca::FPStamped &estimate, orca::Efforts &efforts,
                const std::function<void(double completed, double total)> &send_feedback);
  };

} // namespace orca_base

#endif //ORCA_BASE_PLANNER_HPP
