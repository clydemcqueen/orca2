#ifndef ORCA_BASE_PLANNER_HPP
#define ORCA_BASE_PLANNER_HPP

#include "fiducial_vlam_msgs/msg/map.hpp"

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
  // PlannerBase
  //=====================================================================================

  class PlannerBase
  {
    std::vector<std::shared_ptr<SegmentBase>> segments_;        // Trajectory segments
    std::vector<std::shared_ptr<ControllerBase>> controllers_;  // Trajectory controllers

    int target_idx_;                                            // Current target
    int segment_idx_;                                           // Current segment

    nav_msgs::msg::Path planned_path_;                          // Path for rviz

    void add_keep_station_segment(Pose &plan, double seconds);

    void add_vertical_segment(Pose &plan, double z);

    void add_rotate_segment(Pose &plan, double yaw);

    void add_line_segment(Pose &plan, double x, double y);

    // Plan a trajectory through a series of waypoints
    void plan_trajectory(const std::vector<Pose> &waypoints, const PoseStamped &start);

  protected:

    rclcpp::Logger logger_;
    const BaseContext &cxt_;
    fiducial_vlam_msgs::msg::Map map_;

    std::vector<Pose> targets_;
    bool keep_station_;

    PlannerBase(const rclcpp::Logger &logger, const BaseContext &cxt, fiducial_vlam_msgs::msg::Map map,
                bool keep_station) :
      logger_{logger}, cxt_{cxt}, map_{std::move(map)}, keep_station_{keep_station}, target_idx_{0}, segment_idx_{0}
    {}

    // Plan a trajectory to targets_[target_idx_]
    void plan_trajectory(const PoseStamped &start);

  public:

    const std::vector<Pose> &targets() const
    { return targets_; }

    const nav_msgs::msg::Path &planned_path() const
    { return planned_path_; }

    // Advance the plan, return AdvanceRC
    int advance(double dt, Pose &plan, const nav_msgs::msg::Odometry &estimate, Acceleration &u_bar,
                const std::function<void(double completed, double total)> &send_feedback);
  };

  //=====================================================================================
  // TargetPlanner -- move to target, optionally keeping station
  //=====================================================================================

  class TargetPlanner : public PlannerBase
  {
  public:

    TargetPlanner(const rclcpp::Logger &logger, const BaseContext &cxt, fiducial_vlam_msgs::msg::Map map,
                  const Pose &target, bool keep_station) :
      PlannerBase{logger, cxt, std::move(map), keep_station}
    {
      targets_.push_back(target);
    }
  };

  //=====================================================================================
  // DownSequencePlanner -- markers are on the floor, and there's a down-facing camera
  //=====================================================================================

  class DownSequencePlanner : public PlannerBase
  {
  public:

    DownSequencePlanner(const rclcpp::Logger &logger, const BaseContext &cxt, fiducial_vlam_msgs::msg::Map map,
                        bool random);
  };

  //=====================================================================================
  // ForwardSequencePlanner -- markers are on the walls, and there's a forward-facing camera
  //=====================================================================================

  class ForwardSequencePlanner : public PlannerBase
  {
  public:

    ForwardSequencePlanner(const rclcpp::Logger &logger, const BaseContext &cxt, fiducial_vlam_msgs::msg::Map map,
                           bool random);
  };

} // namespace orca_base

#endif //ORCA_BASE_PLANNER_HPP
