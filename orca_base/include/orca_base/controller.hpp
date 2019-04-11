#ifndef ORCA_BASE_CONTROLLER_HPP
#define ORCA_BASE_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"

#include "base_context.hpp"
#include "model.hpp"
#include "pid.hpp"

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
  OrcaPose prev_target_;
  rclcpp::Time prev_target_time_;
  OrcaPose curr_target_;
  rclcpp::Time curr_target_time_;
  double vx_, vy_, vz_, vyaw_;

  // Feedforward = planned acceleration + acceleration due to drag + acceleration due to buoyancy
  OrcaPose ff_;

public:

  Controller(rclcpp::Logger logger, const BaseContext &cxt);

  // Initialize the controller
  void init(const nav_msgs::msg::Path &path);

  // Advance the controller, return true to continue
  bool advance(const rclcpp::Time &msg_time, const double dt, const OrcaPose &curr, OrcaOdometry &plan, OrcaPose &u_bar);

private:

  // Advance to the next target, return true to continue
  bool set_target(int target);
};

} // namespace orca_base

#endif //ORCA_BASE_CONTROLLER_HPP
