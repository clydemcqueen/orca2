#include <orca_base/geometry.hpp>
#include "orca_base/controller.hpp"
#include "orca_base/planner.hpp"

namespace orca_base {

Controller::Controller(rclcpp::Logger logger, const orca_base::BaseContext &cxt):
  logger_{logger},
  x_controller_{false, cxt.auv_x_pid_kp_, cxt.auv_x_pid_ki_, cxt.auv_x_pid_kd_},
  y_controller_{false, cxt.auv_y_pid_kp_, cxt.auv_y_pid_ki_, cxt.auv_y_pid_kd_},
  z_controller_{false, cxt.auv_z_pid_kp_, cxt.auv_z_pid_ki_, cxt.auv_z_pid_kd_},
  yaw_controller_{true, cxt.auv_yaw_pid_kp_, cxt.auv_yaw_pid_ki_, cxt.auv_yaw_pid_kd_}
{}

void Controller::init(const nav_msgs::msg::Path &path)
{
  path_ = path;

  // path_[0] is the starting position, path_[1] is the first target
  set_target(1);
}

bool Controller::advance(const double dt, const PoseStamped &curr, Acceleration &u_bar)
{
  if (curr.t < trajectory_.p1().t) {
    // Odom pipeline has a lag, ignore old messages
    return true;
  } else if (curr.t > trajectory_.p2().t + STABILIZE) {
    // We hit the time limit for this target
    // TODO don't wait the full STABILIZE time
    if (trajectory_.p2().pose.close_enough(curr.pose)) {
      // Advance to the next target
      return set_target(target_ + 1);
    } else {
      RCLCPP_ERROR(logger_, "didn't reach target");
      return false;
    }
  } else {
    // Get expected position and set PID targets
    auto state = trajectory_.get_state(curr.t);
    x_controller_.set_target(state.pose.x);
    y_controller_.set_target(state.pose.y);
    z_controller_.set_target(state.pose.z);
    yaw_controller_.set_target(state.pose.yaw);

    // Get required acceleration
    u_bar.x = x_controller_.calc(curr.pose.x, dt, state.ff.x);
    u_bar.y = y_controller_.calc(curr.pose.y, dt, state.ff.y);
    u_bar.z = z_controller_.calc(curr.pose.z, dt, state.ff.z);
    u_bar.yaw = yaw_controller_.calc(curr.pose.yaw, dt, state.ff.yaw);

    return true;
  }
}

bool Controller::set_target(int target)
{
  assert(target > 0);
  target_ = target;

  if (target_ >= path_.poses.size()) {
    RCLCPP_INFO(logger_, "mission complete");
    return false;
  }

  // Initialize trajectory
  PoseStamped p1, p2;
  p1.from_msg(path_.poses[target_ - 1]);
  p2.from_msg(path_.poses[target_]);
  p2.t = p2.t - STABILIZE;
  trajectory_.init(p1, p2);

  RCLCPP_INFO(logger_, "target %d position: (%g, %g, %g), yaw %g",
    target_,
    p2.pose.x,
    p2.pose.y,
    p2.pose.z,
    p2.pose.yaw);

  // Initialize PID controllers to previous target
  x_controller_.set_target(p1.pose.x);
  y_controller_.set_target(p1.pose.y);
  z_controller_.set_target(p1.pose.z);
  yaw_controller_.set_target(p1.pose.yaw);

  return true;
}

} // namespace orca_base
