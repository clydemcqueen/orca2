#include "orca_base/motion.hpp"

namespace orca_base {

//=====================================================================================
// Constants
//=====================================================================================

constexpr double EPSILON_PLAN_XYZ = 0.05;       // Close enough for xyz motion (m)
constexpr double EPSILON_PLAN_YAW = M_PI / 90;  // Close enough for yaw motion (r)

//=====================================================================================
// Utilities
//=====================================================================================

// Compute acceleration required to counteract drag force
void drag_force_to_accel_xy(const double yaw, const double x_v, const double y_v, double &x_a, double &y_a)
{
  // Rotate velocity into the body frame
  double forward_v, strafe_v;
  rotate_frame(x_v, y_v, yaw, forward_v, strafe_v);

  // Calc acceleration due to drag force
  double forward_a = force_to_accel(-drag_force_x(forward_v));
  double strafe_a = force_to_accel(-drag_force_y(strafe_v));

  // Rotate back
  rotate_frame(forward_a, strafe_a, -yaw, x_a, y_a);
}

// Compute the deceleration (glide) distance
double deceleration_distance(const double yaw, double velo_x, double velo_y)
{
  constexpr double dt = 0.1;
  double x = 0;
  double y = 0;

  for (double t = 0; t < 10; t += dt) {
    // Compute drag force
    double accel_drag_x, accel_drag_y;
    drag_force_to_accel_xy(yaw, velo_x, velo_y, accel_drag_x, accel_drag_y);

    // Update velocity
    velo_x -= accel_drag_x * dt;
    velo_y -= accel_drag_y * dt;

    // Update distance
    x += velo_x * dt;
    y += velo_y * dt;

    if (std::hypot(velo_x, velo_y) < 0.1) {
      // Close enough
      return std::hypot(x, y);
    }
  }

  return 0;
}

//=====================================================================================
// BaseMotion
//=====================================================================================

BaseMotion::BaseMotion(const rclcpp::Logger &logger, const BaseContext &cxt, const Pose &start, const Pose &goal):
  logger_{logger},
  plan_{start},
  goal_{goal},
  x_controller_{false, cxt.auv_x_pid_kp_, cxt.auv_x_pid_ki_, cxt.auv_x_pid_kd_},
  y_controller_{false, cxt.auv_y_pid_kp_, cxt.auv_y_pid_ki_, cxt.auv_y_pid_kd_},
  z_controller_{false, cxt.auv_z_pid_kp_, cxt.auv_z_pid_ki_, cxt.auv_z_pid_kd_},
  yaw_controller_{true, cxt.auv_yaw_pid_kp_, cxt.auv_yaw_pid_ki_, cxt.auv_yaw_pid_kd_}
{
  twist_ = Twist{};
  ff_ = Acceleration{};

  x_controller_.set_target(start.x);
  y_controller_.set_target(start.y);
  z_controller_.set_target(start.z);
  yaw_controller_.set_target(start.yaw);
}

bool BaseMotion::advance(double dt, const Pose &estimate, Acceleration &u_bar, PoseError &error)
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

VerticalMotion::VerticalMotion(const rclcpp::Logger &logger, const BaseContext &cxt, const Pose &start, const Pose &goal):
  BaseMotion(logger, cxt, start, goal)
{
  assert(start.distance_xy(goal) < EPSILON_PLAN_XYZ && start.distance_yaw(goal) < EPSILON_PLAN_YAW);

  // Ascend (+) or descend (-)
  double direction = goal.z > start.z ? 1 : -1;

  // Target velocity
  twist_.z = direction * cxt.auv_z_speed_;

  // Drag force => thrust force => acceleration => feedforward
  ff_.z = direction * force_to_accel(-drag_force_z(cxt.auv_z_speed_)) + HOVER_ACCEL_Z;

  RCLCPP_INFO(logger_, "vertical: start %g, goal %g, velocity %g, ff %g", start.z, goal.z, twist_.z, ff_.z);
}

bool VerticalMotion::advance(double dt, const Pose &estimate, Acceleration &u_bar, PoseError &error)
{
  if (goal_.distance_z(plan_) > EPSILON_PLAN_XYZ) {
    // Update plan
    plan_.z += twist_.z * dt;

    // Set target
    z_controller_.set_target(plan_.z);

    // Accumulate error
    error.plan = plan_; // TODO
    error.estimate = estimate; // TODO
    error.add_error();

    // Compute u_bar
    return BaseMotion::advance(dt, estimate, u_bar, error);
  } else {
    finish(u_bar);
    return false;
  }
}

//=====================================================================================
// RotateMotion
//=====================================================================================

RotateMotion::RotateMotion(const rclcpp::Logger &logger, const BaseContext &cxt, const Pose &start, const Pose &goal):
  BaseMotion(logger, cxt, start, goal)
{
  assert(start.distance_xy(goal) < EPSILON_PLAN_XYZ || start.distance_z(goal) < EPSILON_PLAN_XYZ);

  // Pick the shortest direction
  twist_.yaw = norm_angle(goal.yaw - start.yaw) > 0 ? cxt.auv_yaw_speed_ : -cxt.auv_yaw_speed_;

  // Drag torque => thrust torque => acceleration => feedforward
  ff_.yaw = torque_to_accel_yaw(-drag_torque_yaw(twist_.yaw));
  ff_.z = HOVER_ACCEL_Z;

  RCLCPP_INFO(logger_, "rotate: start %g, goal %g, velocity %g, accel %g", start.yaw, goal.yaw, twist_.yaw, ff_.yaw);
}

bool RotateMotion::advance(double dt, const Pose &estimate, Acceleration &u_bar, PoseError &error)
{
  if (goal_.distance_yaw(plan_) > EPSILON_PLAN_YAW) {
    // Update plan
    plan_.yaw = norm_angle(plan_.yaw + twist_.yaw * dt);

    // Set target
    yaw_controller_.set_target(plan_.yaw);

    // Accumulate error
    error.plan = plan_; // TODO
    error.estimate = estimate; // TODO
    error.add_error();

    // Compute u_bar
    return BaseMotion::advance(dt, estimate, u_bar, error);
  } else {
    finish(u_bar);
    return false;
  }
}

//=====================================================================================
// LineMotion
//=====================================================================================

LineMotion::LineMotion(const rclcpp::Logger &logger, const BaseContext &cxt, const Pose &start, const Pose &goal):
  BaseMotion(logger, cxt, start, goal)
{
  assert(start.distance_z(goal) < EPSILON_PLAN_XYZ && start.distance_yaw(goal) < EPSILON_PLAN_YAW);

  // Compute angle start => goal
  double angle_to_goal = atan2(goal.y - start.y, goal.x - start.x);

  // Drag force => thrust force => acceleration => feedforward
  drag_force_to_accel_xy(goal.yaw, cxt.auv_xy_speed_ * cos(angle_to_goal), cxt.auv_xy_speed_ * sin(angle_to_goal), ff_.x, ff_.y);
  ff_.z = HOVER_ACCEL_Z;

  RCLCPP_INFO(logger_, "line: start (%g, %g, %g), goal (%g, %g, %g), ff (%g, %g, %g)",
    start.x, start.y, start.z, goal.x, goal.y, goal.z, ff_.x, ff_.y, ff_.z);
}

bool LineMotion::advance(double dt, const Pose &estimate, Acceleration &u_bar, PoseError &error)
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

    // Accumulate error
    error.plan = plan_; // TODO
    error.estimate = estimate; // TODO

    // Compute u_bar
    return BaseMotion::advance(dt, estimate, u_bar, error);
  } else {
    finish(u_bar);
    return false;
  }
}

} // namespace orca_base