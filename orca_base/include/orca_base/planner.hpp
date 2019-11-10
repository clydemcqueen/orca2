#ifndef ORCA_BASE_PLANNER_HPP
#define ORCA_BASE_PLANNER_HPP

#include "fiducial_vlam_msgs/msg/map.hpp"

#include "orca_base/segment.hpp"

namespace orca_base
{

  //=====================================================================================
  // PlannerBase
  //=====================================================================================

  class PlannerBase
  {
    std::vector<std::shared_ptr<SegmentBase>> segments_;        // Trajectory segments
    std::vector<std::shared_ptr<ControllerBase>> controllers_;  // Trajectory controllers
    int segment_idx_;                                           // Current segment
    nav_msgs::msg::Path planned_path_;                          // Path for rviz

    void add_keep_station_segment(Pose &plan, double seconds);

    void add_vertical_segment(Pose &plan, double z);

    void add_rotate_segment(Pose &plan, double yaw);

    void add_line_segment(Pose &plan, double x, double y);

  protected:

    rclcpp::Logger logger_;
    const BaseContext &cxt_;
    fiducial_vlam_msgs::msg::Map map_;

    explicit PlannerBase(const rclcpp::Logger &logger, const BaseContext &cxt, fiducial_vlam_msgs::msg::Map map) :
      logger_{logger}, cxt_{cxt}, map_{std::move(map)}, segment_idx_{}
    {}

    void plan_target(const Pose &target, const PoseStamped &start, bool keep_station);

    void plan_waypoints(const std::vector<Pose> &waypoints, const PoseStamped &start, bool keep_station);

    virtual void plan_segments(const PoseStamped &start) = 0;

  public:

    const std::vector<std::shared_ptr<SegmentBase>> &segments() const
    { return segments_; }

    const nav_msgs::msg::Path &planned_path() const
    { return planned_path_; }

    // Advance the plan, return true to continue
    bool advance(double dt, Pose &plan, const nav_msgs::msg::Odometry &estimate, Acceleration &u_bar,
                 const std::function<void(double completed, double total)>& send_feedback);
  };

  //=====================================================================================
  // KeepStationPlanner -- keep current pose
  //=====================================================================================

  class KeepStationPlanner : public PlannerBase
  {
  public:

    explicit KeepStationPlanner(const rclcpp::Logger &logger, const BaseContext &cxt, fiducial_vlam_msgs::msg::Map map) : PlannerBase{logger, cxt, std::move(map)}
    {}

    void plan_segments(const PoseStamped &start) override;
  };

  //=====================================================================================
  // KeepOriginPlanner -- keep station directly below the origin
  //=====================================================================================

  class KeepOriginPlanner : public PlannerBase
  {
  public:

    explicit KeepOriginPlanner(const rclcpp::Logger &logger, const BaseContext &cxt, fiducial_vlam_msgs::msg::Map map) : PlannerBase{logger, cxt, std::move(map)}
    {}

    void plan_segments(const PoseStamped &start) override;
  };

  //=====================================================================================
  // DownSequencePlanner -- markers are on the floor, and there's a down-facing camera
  //=====================================================================================

  class DownSequencePlanner : public PlannerBase
  {
    bool random_;

  public:

    explicit DownSequencePlanner(const rclcpp::Logger &logger, const BaseContext &cxt, fiducial_vlam_msgs::msg::Map map, bool random) :
      PlannerBase{logger, cxt, std::move(map)},
      random_{random}
    {}

    void plan_segments(const PoseStamped &start) override;
  };

  //=====================================================================================
  // ForwardSequencePlanner -- markers are on the walls, and there's a forward-facing camera
  //=====================================================================================

  class ForwardSequencePlanner : public PlannerBase
  {
    bool random_;

  public:

    explicit ForwardSequencePlanner(const rclcpp::Logger &logger, const BaseContext &cxt, fiducial_vlam_msgs::msg::Map map, bool random) :
      PlannerBase{logger, cxt, std::move(map)},
      random_{random}
    {}

    void plan_segments(const PoseStamped &start) override;
  };

} // namespace orca_base

#endif //ORCA_BASE_PLANNER_HPP
