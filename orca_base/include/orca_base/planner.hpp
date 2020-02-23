#ifndef ORCA_BASE_PLANNER_HPP
#define ORCA_BASE_PLANNER_HPP

#include "image_geometry/pinhole_camera_model.h"

#include "orca_description/parser.hpp"
#include "orca_msgs/msg/control.hpp"
#include "orca_shared/geometry.hpp"

#include "orca_base/map.hpp"
#include "orca_base/segment.hpp"

namespace orca_base
{

  //=====================================================================================
  // AdvanceRC
  //=====================================================================================

  struct AdvanceRC
  {
    static constexpr int CONTINUE = 0;
    static constexpr int SUCCESS = 1;
    static constexpr int FAILURE = 2;
  };

  //=====================================================================================
  // PlannerStatus
  //=====================================================================================

  struct PlannerStatus
  {
    int targets_total{};          // Number of targets in global plan
    int target_idx{};             // Current target index
    int target_marker_id{};       // Current target marker id

    uint8_t planner{};            // Local planner running, see Control.msg for values

    int segments_total{};         // Number of segments in local plan
    int segment_idx{};            // Current segment index
    std::string segment_info;     // Current segment info string
  };

  //=====================================================================================
  // Target
  //=====================================================================================

  struct Target
  {
    int marker_id;
    FP fp; // Hmmm... I think we can get by with just an orca::Pose

    Target() : marker_id{NOT_A_MARKER}
    {}

    Target(int _marker_id, FP _fp) : marker_id{_marker_id}, fp{std::move(_fp)}
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
    std::shared_ptr<PoseController> controller_;                // Motion controller
    nav_msgs::msg::Path local_path_;                            // Path to next target

    // Add a trajectory segment and update plan
    void add_keep_station_segment(FPStamped &plan, double seconds);

    void add_vertical_segment(FPStamped &plan, double z);

    void add_rotate_segment(FPStamped &plan, double yaw);

    void add_line_segment(FPStamped &plan, double x, double y);

    void add_pose_segment(FPStamped &plan, const FP &goal);

  public:

    LocalPlanner(const rclcpp::Logger &logger, const BaseContext &cxt, const FPStamped &start, Target target, Map map,
                 bool keep_station, PlannerStatus &status);

    bool advance(const rclcpp::Duration &d, FPStamped &plan, const FPStamped &estimate, orca::Efforts &efforts,
                 PlannerStatus &status);

    // TODO publish local path
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
    std::shared_ptr<ObservationController> controller_;               // Motion controller

  public:

    MoveToMarkerPlanner(const rclcpp::Logger &logger, const BaseContext &cxt, const ObservationStamped &start,
                        PlannerStatus &status);

    bool advance(const rclcpp::Duration &d, FPStamped &plan, const FPStamped &estimate,
                 orca::Efforts &efforts, PlannerStatus &status);
  };

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
#if 0
    FPStamped plan_pose_;                                     // Planned pose
    orca::Twist plan_twist_;                                  // Planned twist
#endif

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

#if 0
    const FPStamped &plan_pose() const
    { return plan_pose_; }

    const orca::Twist &plan_twist() const
    { return plan_twist_; }
#endif

    const nav_msgs::msg::Path &global_path() const
    { return global_path_; }

    // Advance the plan by dt, return AdvanceRC
    int advance(const rclcpp::Duration &d, FPStamped &plan, const FPStamped &estimate, orca::Efforts &efforts,
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

#endif //ORCA_BASE_PLANNER_HPP
