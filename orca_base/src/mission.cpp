#include "orca_base/mission.hpp"

namespace orca_base
{

  Mission::Mission(rclcpp::Logger logger, std::shared_ptr<BasePlanner> planner, const BaseContext &cxt,
                   const fiducial_vlam_msgs::msg::Map &map, const PoseStamped &start) :
    logger_{logger},
    planner_{std::move(planner)}
  {
    // Create path
    planner_->plan(map, start);

    // Start
    segment_idx_ = 0;
    RCLCPP_INFO(logger_, "mission has %d segment(s), segment 0", planner_->segments().size());
  }

  bool Mission::advance(double dt, Pose &plan, Acceleration &ff)
  {
    if (planner_->segments()[segment_idx_]->advance(dt)) {
      plan = planner_->segments()[segment_idx_]->plan();
      ff = planner_->segments()[segment_idx_]->ff();
      return true;
    }

    if (++segment_idx_ < planner_->segments().size()) {
      RCLCPP_INFO(logger_, "mission segment %d", segment_idx_);
      plan = planner_->segments()[segment_idx_]->plan();
      ff = planner_->segments()[segment_idx_]->ff();
      return true;
    }

    RCLCPP_INFO(logger_, "mission complete");
    return false;
  }

} // namespace orca_base
