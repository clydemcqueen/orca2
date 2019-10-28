#include "orca_base/mission.hpp"

namespace orca_base
{

  Mission::Mission(rclcpp::Logger logger, std::shared_ptr<BasePlanner> planner, const BaseContext &cxt,
                   const fiducial_vlam_msgs::msg::Map &map, const PoseStamped &start) :
    logger_{logger},
    planner_{std::move(planner)}
  {
    RCLCPP_INFO(logger, "pid x=(%g, %g, %g), y=(%g, %g, %g), z=(%g, %g, %g), yaw=(%g, %g, %g)",
                cxt.auv_x_pid_kp_, cxt.auv_x_pid_ki_, cxt.auv_x_pid_kd_,
                cxt.auv_y_pid_kp_, cxt.auv_y_pid_ki_, cxt.auv_y_pid_kd_,
                cxt.auv_z_pid_kp_, cxt.auv_z_pid_ki_, cxt.auv_z_pid_kd_,
                cxt.auv_yaw_pid_kp_, cxt.auv_yaw_pid_ki_, cxt.auv_yaw_pid_kd_);

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
