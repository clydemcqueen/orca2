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

bool Controller::advance(const rclcpp::Time &msg_time, const double dt, const OrcaPose &curr, OrcaOdometry &plan, OrcaPose &u_bar)
{
  // TODO use curr.time instead of msg_time

  if (msg_time < prev_target_time_) {

    // As we shift control from joy pipeline to odom pipeline we're likely to see old odom messages,
    // as odom pipeline has a lag. Ignore these.
    return true;

  } else if (msg_time > curr_target_time_ + STABILIZE) {

    // We hit the time limit for this target
    if (curr_target_.close_enough(curr)) {
      // We're there: advance to the next target
      return set_target(target_ + 1);
    } else {
      // We missed: abort mission
      RCLCPP_ERROR(logger_, "didn't reach target");
      return false;
    }

  } else {

    // Compute expected position and set PID targets
    if (msg_time < curr_target_time_) {
      auto elapsed_time = (msg_time - prev_target_time_).seconds();
      x_controller_.set_target(prev_target_.x + vx_ * elapsed_time);
      y_controller_.set_target(prev_target_.y + vy_ * elapsed_time);
      z_controller_.set_target(prev_target_.z + vz_ * elapsed_time);
      yaw_controller_.set_target(norm_angle(prev_target_.yaw + vyaw_ * elapsed_time));
    }

    // Compute acceleration
    u_bar.x = x_controller_.calc(curr.x, dt, ff_.x);
    u_bar.y = y_controller_.calc(curr.y, dt, ff_.y);
    u_bar.z = z_controller_.calc(curr.z, dt, ff_.z);
    u_bar.yaw = yaw_controller_.calc(curr.yaw, dt, ff_.yaw);

    // Update pose covariance TODO revisit
    for (int i = 0; i < 4; ++i) {
      plan.pose_covariance[i + 4 * i] += DEF_COVAR * dt;
    }

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

  prev_target_.from_msg(path_.poses[target_ - 1]);
  prev_target_time_ = rclcpp::Time(path_.poses[target_ - 1].header.stamp);

  curr_target_.from_msg(path_.poses[target_]);
  curr_target_time_ = rclcpp::Time(path_.poses[target_].header.stamp) - STABILIZE;

  RCLCPP_INFO(logger_, "target %d position: (%g, %g, %g), yaw %g",
    target_,
    curr_target_.x,
    curr_target_.y,
    curr_target_.z,
    curr_target_.yaw);

  auto dive_time = (curr_target_time_ - prev_target_time_).seconds();
  assert(dive_time > 0);

  // Velocity vector from previous target to this target
  vx_ = (curr_target_.x - prev_target_.x) / dive_time;
  vy_ = (curr_target_.y - prev_target_.y) / dive_time;
  vz_ = (curr_target_.z - prev_target_.z) / dive_time;
  vyaw_ = norm_angle(curr_target_.yaw - prev_target_.yaw) / dive_time;

  // Calc ff
  ff_ = OrcaPose{}; // TODO calc feedforward

  RCLCPP_INFO(logger_, "target %d velocity: (%g, %g, %g), yaw %g", target_, vx_, vy_, vz_, vyaw_);

  // Initialize PID controllers to previous target
  x_controller_.set_target(prev_target_.x);
  y_controller_.set_target(prev_target_.y);
  z_controller_.set_target(prev_target_.z);
  yaw_controller_.set_target(prev_target_.yaw);

  return true;
}

} // namespace orca_base
