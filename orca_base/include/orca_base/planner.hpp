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
    orca::FP fp; // Hmmm... I think we can get by with just an orca::Pose

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

    bool advance(const rclcpp::Duration &d, orca::FPStamped &plan, const orca::FPStamped &estimate, orca::Pose &error,
                 orca::Efforts &efforts, const std::function<void(double completed, double total)> &send_feedback);

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
    std::shared_ptr<ObservationController> controller_;               // Motion controller

  public:

    MoveToMarkerPlanner(const rclcpp::Logger &logger, const BaseContext &cxt, const orca::Observation &start);

    bool advance(const rclcpp::Duration &d, orca::FPStamped &plan, const orca::FPStamped &estimate, orca::Pose &error,
                 orca::Efforts &efforts, const std::function<void(double completed, double total)> &send_feedback);

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

    // Create a local_planner_
    void start_local_plan(const orca::FPStamped &start);

    // Create a recovery_planner_
    void start_recovery_plan(const orca::Observation &start);

  public:

    explicit MissionPlanner(const rclcpp::Logger &logger, const BaseContext &cxt, Map map,
                            orca_description::Parser parser,
                            const image_geometry::PinholeCameraModel &fcam_model, bool keep_station,
                            std::vector<Target> targets);

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

    // Advance the plan by dt, return AdvanceRC
    int advance(const rclcpp::Duration &d, orca::FPStamped &plan, const orca::FPStamped &estimate, orca::Pose &error,
                orca::Efforts &efforts, const std::function<void(double completed, double total)> &send_feedback);

    // Factory: move to a pose and optionally keep station
    static std::shared_ptr<MissionPlanner>
    plan_pose(const rclcpp::Logger &logger, const BaseContext &cxt, const Map &map,
              const orca_description::Parser &parser,
              const image_geometry::PinholeCameraModel &fcam_model, const orca::FP &fp, bool keep_station);

    // Factory: visit all markers in sequence or at random
    // Markers must be on the floor, facing up, and there must be a down-facing camera
    static std::shared_ptr<MissionPlanner>
    plan_floor_markers(const rclcpp::Logger &logger, const BaseContext &cxt, const Map &map,
                       const orca_description::Parser &parser,
                       const image_geometry::PinholeCameraModel &fcam_model, bool random);

    // Factory: visit all markers in sequence or at random
    // Markers must be on the wall and there must be a forward-facing camera
    static std::shared_ptr<MissionPlanner>
    plan_wall_markers(const rclcpp::Logger &logger, const BaseContext &cxt, const Map &map,
                      const orca_description::Parser &parser,
                      const image_geometry::PinholeCameraModel &fcam_model, bool random);

    // Factory: move through a series of poses
    static std::shared_ptr<MissionPlanner>
    plan_msgs(const rclcpp::Logger &logger, const BaseContext &cxt, const Map &map,
              const orca_description::Parser &parser,
              const image_geometry::PinholeCameraModel &fcam_model, const std::vector<geometry_msgs::msg::Pose> &msgs,
              bool keep_station);
  };

} // namespace orca_base

#endif //ORCA_BASE_PLANNER_HPP
