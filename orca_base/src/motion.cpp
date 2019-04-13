#include "orca_base/motion.hpp"

namespace orca_base {

//=====================================================================================
// BaseMotion
//=====================================================================================

BaseMotion::BaseMotion(rclcpp::Logger &logger, const BaseContext &cxt, const Pose &start, const Pose &goal):
  x_controller_{false, cxt.auv_x_pid_kp_, cxt.auv_x_pid_ki_, cxt.auv_x_pid_kd_},
  y_controller_{false, cxt.auv_y_pid_kp_, cxt.auv_y_pid_ki_, cxt.auv_y_pid_kd_},
  z_controller_{false, cxt.auv_z_pid_kp_, cxt.auv_z_pid_ki_, cxt.auv_z_pid_kd_},
  yaw_controller_{true, cxt.auv_yaw_pid_kp_, cxt.auv_yaw_pid_ki_, cxt.auv_yaw_pid_kd_}
{
  plan_ = start;
  goal_ = goal;
  twist_ = Twist{};
  ff_ = Acceleration{};

  x_controller_.set_target(start.x);
  y_controller_.set_target(start.y);
  z_controller_.set_target(start.z);
  yaw_controller_.set_target(start.yaw);
}

bool BaseMotion::advance(double dt, const Pose &estimate, Acceleration &u_bar)
{
  u_bar.x = x_controller_.calc(estimate.x, dt, ff_.x);
  u_bar.y = y_controller_.calc(estimate.y, dt, ff_.y);
  u_bar.z = z_controller_.calc(estimate.z, dt, ff_.z);
  u_bar.yaw = yaw_controller_.calc(estimate.yaw, dt, ff_.yaw);
  return true;
}

void BaseMotion::finish(Acceleration &u_bar)
{
  plan_ = goal_;
  twist_ = Twist{};
  u_bar = Acceleration{};
}

//=====================================================================================
// VerticalMotion
//=====================================================================================

VerticalMotion::VerticalMotion(rclcpp::Logger &logger, const BaseContext &cxt, const Pose &start, const Pose &goal):
  BaseMotion(logger, cxt, start, goal)
{
  assert(start.distance_xy(goal) < EPSILON_PLAN_XYZ && start.distance_yaw(goal) < EPSILON_PLAN_YAW);

  // Ascend (+) or descend (-)
  double direction = goal.z > start.z ? 1 : -1;

  // Target velocity
  twist_.z = direction * VELO_Z;

  // Drag force => thrust force => acceleration => feedforward
  ff_.z = direction * force_to_accel_z(-drag_force_z(VELO_Z)) + HOVER_ACCEL_Z;

  RCLCPP_INFO(logger, "vertical: start %g, goal %g, velocity %g, ff %g", start.z, goal.z, twist_.z, ff_.z);
}

bool VerticalMotion::advance(double dt, const Pose &estimate, Acceleration &u_bar)
{
  if (goal_.distance_z(plan_) > EPSILON_PLAN_XYZ) {
    // Update plan
    plan_.z += twist_.z * dt;

    // Set target
    z_controller_.set_target(plan_.z);

    // Compute u_bar
    return BaseMotion::advance(dt, estimate, u_bar);
  } else {
    finish(u_bar);
    return false;
  }
}

//=====================================================================================
// RotateMotion
//=====================================================================================

RotateMotion::RotateMotion(rclcpp::Logger &logger, const BaseContext &cxt, const Pose &start, const Pose &goal):
  BaseMotion(logger, cxt, start, goal)
{
  assert(start.distance_xy(goal) < EPSILON_PLAN_XYZ || start.distance_z(goal) < EPSILON_PLAN_XYZ);

  // Pick the shortest direction
  twist_.yaw = norm_angle(goal.yaw - start.yaw) > 0 ? VELO_YAW : -VELO_YAW;

  // Drag torque => thrust torque => acceleration => feedforward
  ff_.yaw = torque_to_accel_yaw(-drag_torque_yaw(twist_.yaw));
  ff_.z = HOVER_ACCEL_Z;

  RCLCPP_INFO(logger, "rotate: start %g, goal %g, velocity %g, accel %g", start.yaw, goal.yaw, twist_.yaw, ff_.yaw);
}

bool RotateMotion::advance(double dt, const Pose &estimate, Acceleration &u_bar)
{
  if (goal_.distance_yaw(plan_) > EPSILON_PLAN_YAW) {
    // Update plan
    plan_.yaw = norm_angle(plan_.yaw + twist_.yaw * dt);

    // Set target
    yaw_controller_.set_target(plan_.yaw);

    // Compute u_bar
    return BaseMotion::advance(dt, estimate, u_bar);
  } else {
    finish(u_bar);
    return false;
  }
}

//=====================================================================================
// LineMotion
//=====================================================================================

LineMotion::LineMotion(rclcpp::Logger &logger, const BaseContext &cxt, const Pose &start, const Pose &goal):
  BaseMotion(logger, cxt, start, goal)
{
  assert(start.distance_z(goal) < EPSILON_PLAN_XYZ && start.distance_yaw(goal) < EPSILON_PLAN_YAW);

  // Compute angle start => goal
  double angle_to_goal = atan2(goal.y - start.y, goal.x - start.x);

  // Drag force => thrust force => acceleration => feedforward
  drag_force_to_accel_xy(goal.yaw, VELO_XY * cos(angle_to_goal), VELO_XY * sin(angle_to_goal), ff_.x, ff_.y);
  ff_.z = HOVER_ACCEL_Z;

  RCLCPP_INFO(logger, "line: start (%g, %g, %g), goal (%g, %g, %g), ff (%g, %g, %g)",
    start.x, start.y, start.z, goal.x, goal.y, goal.z, ff_.x, ff_.y, ff_.z);
}

bool LineMotion::advance(double dt, const Pose &estimate, Acceleration &u_bar)
{
  double distance_remaining = goal_.distance_xy(plan_);
  if (distance_remaining > EPSILON_PLAN_XYZ) {
    if (distance_remaining - deceleration_distance(goal_.yaw, twist_.x, twist_.y) < EPSILON_PLAN_XYZ) {
      // Decelerate
      ff_.x = ff_.y = 0;
    }

    // Compute acceleration due to drag
    double accel_drag_x, accel_drag_y;
    drag_force_to_accel_xy(goal_.yaw, twist_.x, twist_.y, accel_drag_x, accel_drag_y);

    // Update velocity
    twist_.x += (ff_.x - accel_drag_x) * dt;
    twist_.y += (ff_.y - accel_drag_y) * dt;

    // Update plan
    plan_.x += twist_.x * dt;
    plan_.y += twist_.y * dt;

    // Set targets
    x_controller_.set_target(plan_.x);
    y_controller_.set_target(plan_.y);

    // Compute u_bar
    return BaseMotion::advance(dt, estimate, u_bar);
  } else {
    finish(u_bar);
    return false;
  }
}

} // namespace orca_base