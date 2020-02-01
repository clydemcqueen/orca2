#ifndef ORCA_BASE_PLANNER_HPP
#define ORCA_BASE_PLANNER_HPP

#include "image_geometry/pinhole_camera_model.h"

#include "orca_description/parser.hpp"
#include "orca_shared/geometry.hpp"

#include "orca_base/map.hpp"
#include "orca_base/segment.hpp"

namespace orca_base
{

  class Planner
  {
  public:

    struct AdvanceRC
    {
      static constexpr int CONTINUE = 0;
      static constexpr int SUCCESS = 1;
      static constexpr int FAILURE = 2;
    };

  private:

    struct Target
    {
      int marker_id;
      orca::FP fp;

      Target() : marker_id{orca::NOT_A_MARKER}
      {}

      Target(int _marker_id, orca::FP _fp) : marker_id{_marker_id}, fp{std::move(_fp)}
      {}
    };

    rclcpp::Logger logger_;
    const BaseContext &cxt_;
    Map map_;
    orca_description::Parser parser_;
    image_geometry::PinholeCameraModel fcam_model_;

    // Plan & plan state
    std::vector<Target> targets_;                               // Global plan
    bool keep_station_;                                         // Keep station (pause forever) at last target
    std::vector<std::shared_ptr<PoseSegmentBase>> segments_;    // Local plan
    std::shared_ptr<PoseController> controller_;                // Plan controller
    int target_idx_;                                            // Current target
    int segment_idx_;                                           // Current segment

    // Recovery state
    bool recovery_;
    std::shared_ptr<MoveToMarkerSegment> recovery_segment_;
    std::shared_ptr<MoveToMarkerController> recovery_controller_;

    nav_msgs::msg::Path planned_path_;                          // Path to next target
    nav_msgs::msg::Path target_path_;                           // Path to all targets

    void add_keep_station_segment(orca::FP &plan, double seconds);

    void add_vertical_segment(orca::FP &plan, double z);

    void add_rotate_segment(orca::FP &plan, double yaw);

    void add_line_segment(orca::FP &plan, double x, double y);

    // Local planner: plan a trajectory through a series of waypoints
    void plan_trajectory(const std::vector<orca::Pose> &waypoints, const orca::FP &start);

    // Local planner: plan a trajectory to targets_[target_idx_]
    void plan_trajectory(const orca::FP &start);

    // Attempt to find a good pose
    void recover(const orca::Observation &start);

    // Given a planned pose, predict the marker observations for the forward camera
    void predict_observations(orca::FP &plan) const;

    // Create target_path_ for subsequent publishing
    void create_target_path();

    int advance_recovery(double dt, orca::FP &plan, const orca::FP &estimate, orca::Efforts &efforts,
                         const std::function<void(double completed, double total)> &send_feedback);

    int advance_plan(double dt, orca::FP &plan, const orca::FP &estimate, orca::Efforts &efforts,
                     const std::function<void(double completed, double total)> &send_feedback);

  public:

    Planner(const rclcpp::Logger &logger, const BaseContext &cxt, Map map, orca_description::Parser parser,
            const image_geometry::PinholeCameraModel &fcam_model_);

    const std::vector<Target> &targets() const
    { return targets_; }

    const nav_msgs::msg::Path &planned_path() const
    { return planned_path_; }

    const nav_msgs::msg::Path &target_path() const
    { return target_path_; }

    // Global planner: move to a pose, optionally keeping station when we get there
    void plan_target(const orca::FP &fp, bool keep_station);

    // Global planner: visit all markers on the floor (for down-facing camera(s))
    void plan_floor_markers(bool random);

    // Global planner: visit all markers on the wall (for a forward-facing camera)
    void plan_wall_markers(bool random);

    // Advance the plan by dt, return AdvanceRC
    int advance(double dt, orca::FP &plan, const orca::FP &estimate, orca::Efforts &efforts,
                const std::function<void(double completed, double total)> &send_feedback);
  };

} // namespace orca_base

#endif //ORCA_BASE_PLANNER_HPP
