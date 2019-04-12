#ifndef ORCA_BASE_CONTROLLER_HPP
#define ORCA_BASE_CONTROLLER_HPP

#include "nav_msgs/msg/path.hpp"

#include "orca_base/base_context.hpp"
#include "orca_base/pid.hpp"
#include "orca_base/trajectory.hpp"

namespace orca_base {

class Controller
{
  // ROS logger
  rclcpp::Logger logger_;

  // PID controllers
  pid::Controller x_controller_;
  pid::Controller y_controller_;
  pid::Controller z_controller_;
  pid::Controller yaw_controller_;

  // Path to follow
  nav_msgs::msg::Path path_;
  int target_;
  Trajectory trajectory_;

public:

  Controller(rclcpp::Logger logger, const BaseContext &cxt);

  // Initialize the controller
  void init(const nav_msgs::msg::Path &path);

  // Advance the controller, return true to continue
  bool advance(const double dt, const PoseStamped &curr, Acceleration &u_bar);

private:

  // Advance to the next target, return true to continue
  bool set_target(int target);
};

} // namespace orca_base

#endif //ORCA_BASE_CONTROLLER_HPP
