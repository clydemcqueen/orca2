#ifndef ORCA_BASE_PLANNER_HPP
#define ORCA_BASE_PLANNER_HPP

#include "fiducial_vlam_msgs/msg/map.hpp"

#include "orca_base/segment.hpp"

namespace orca_base
{

  //=====================================================================================
  // BasePlanner
  //=====================================================================================

  class BasePlanner
  {
    std::vector<std::shared_ptr<BaseSegment>> segments_;  // Trajectory segments
    nav_msgs::msg::Path planned_path_;                    // Path for rviz

    void add_vertical_segment(Pose &plan, double z);

    void add_rotate_segment(Pose &plan, double yaw);

    void add_line_segment(Pose &plan, double x, double y);

  protected:

    rclcpp::Logger logger_;
    const BaseContext &cxt_;

    explicit BasePlanner(const rclcpp::Logger &logger, const BaseContext &cxt) :
      logger_{logger}, cxt_{cxt}
    {}

    void plan_target(const Pose &target, const PoseStamped &start);

    void plan_waypoints(const std::vector<Pose> &waypoints, const PoseStamped &start);

  public:

    const std::vector<std::shared_ptr<BaseSegment>> &segments() const
    { return segments_; }

    const nav_msgs::msg::Path &planned_path() const
    { return planned_path_; }

    virtual void plan(const fiducial_vlam_msgs::msg::Map &map, const PoseStamped &start) = 0;
  };

  //=====================================================================================
  // KeepStationPlanner -- keep current pose
  //=====================================================================================

  class KeepStationPlanner : public BasePlanner
  {
  public:

    explicit KeepStationPlanner(const rclcpp::Logger &logger, const BaseContext &cxt) : BasePlanner{logger, cxt}
    {}

    void plan(const fiducial_vlam_msgs::msg::Map &map, const PoseStamped &start) override;
  };

  //=====================================================================================
  // KeepOriginPlanner -- keep station directly below the origin
  //=====================================================================================

  class KeepOriginPlanner : public BasePlanner
  {
  public:

    explicit KeepOriginPlanner(const rclcpp::Logger &logger, const BaseContext &cxt) : BasePlanner{logger, cxt}
    {}

    void plan(const fiducial_vlam_msgs::msg::Map &map, const PoseStamped &start) override;
  };

  //=====================================================================================
  // DownSequencePlanner -- markers are on the floor, and there's a down-facing camera
  //=====================================================================================

  class DownSequencePlanner : public BasePlanner
  {
    bool random_;

  public:

    explicit DownSequencePlanner(const rclcpp::Logger &logger, const BaseContext &cxt, bool random) :
      BasePlanner{logger, cxt},
      random_{random}
    {}

    void plan(const fiducial_vlam_msgs::msg::Map &map, const PoseStamped &start) override;
  };

  //=====================================================================================
  // ForwardSequencePlanner -- markers are on the walls, and there's a forward-facing camera
  //=====================================================================================

  class ForwardSequencePlanner : public BasePlanner
  {
    bool random_;

  public:

    explicit ForwardSequencePlanner(const rclcpp::Logger &logger, const BaseContext &cxt, bool random) :
      BasePlanner{logger, cxt},
      random_{random}
    {}

    void plan(const fiducial_vlam_msgs::msg::Map &map, const PoseStamped &start) override;
  };

  //=====================================================================================
  // Body planners move back/forth along a body axis, for testing motion on that axis
  // X = forward/back, Y = left/right, Z = up/down, Yaw = ccw/cw
  //=====================================================================================

  class BodyXPlanner : public BasePlanner
  {
  public:

    explicit BodyXPlanner(const rclcpp::Logger &logger, const BaseContext &cxt) : BasePlanner{logger, cxt}
    {}

    void plan(const fiducial_vlam_msgs::msg::Map &map, const PoseStamped &start) override;
  };

  class BodyYPlanner : public BasePlanner
  {
  public:

    explicit BodyYPlanner(const rclcpp::Logger &logger, const BaseContext &cxt) : BasePlanner{logger, cxt}
    {}

    void plan(const fiducial_vlam_msgs::msg::Map &map, const PoseStamped &start) override;
  };

  class BodyZPlanner : public BasePlanner
  {
  public:

    explicit BodyZPlanner(const rclcpp::Logger &logger, const BaseContext &cxt) : BasePlanner{logger, cxt}
    {}

    void plan(const fiducial_vlam_msgs::msg::Map &map, const PoseStamped &start) override;
  };

  class BodyYawPlanner : public BasePlanner
  {
  public:

    explicit BodyYawPlanner(const rclcpp::Logger &logger, const BaseContext &cxt) : BasePlanner{logger, cxt}
    {}

    void plan(const fiducial_vlam_msgs::msg::Map &map, const PoseStamped &start) override;
  };

} // namespace orca_base

#endif //ORCA_BASE_PLANNER_HPP
