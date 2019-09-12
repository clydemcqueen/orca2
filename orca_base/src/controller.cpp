#include "orca_base/controller.hpp"

namespace orca_base
{

  BaseController::BaseController(const BaseContext &cxt) :
    x_controller_{false, cxt.auv_x_pid_kp_, cxt.auv_x_pid_ki_, cxt.auv_x_pid_kd_},
    y_controller_{false, cxt.auv_y_pid_kp_, cxt.auv_y_pid_ki_, cxt.auv_y_pid_kd_},
    z_controller_{false, cxt.auv_z_pid_kp_, cxt.auv_z_pid_ki_, cxt.auv_z_pid_kd_},
    yaw_controller_{true, cxt.auv_yaw_pid_kp_, cxt.auv_yaw_pid_ki_, cxt.auv_yaw_pid_kd_}
  {
  }

  void BaseController::calc(const BaseContext &cxt, double dt, const Pose &plan, const Pose &estimate,
                            const Acceleration &ff, Acceleration &u_bar)
  {
    // Set targets
    x_controller_.set_target(plan.x);
    y_controller_.set_target(plan.y);
    z_controller_.set_target(plan.z);
    yaw_controller_.set_target(plan.yaw);

    // Calculate response to the error
    u_bar.x = x_controller_.calc(estimate.x, dt) + ff.x;
    u_bar.y = y_controller_.calc(estimate.y, dt) + ff.y;
    u_bar.z = z_controller_.calc(estimate.z, dt) + ff.z;
    u_bar.yaw = yaw_controller_.calc(estimate.yaw, dt) + ff.yaw;
  }

  void DeadzoneController::calc(const BaseContext &cxt, double dt, const Pose &plan, const Pose &estimate,
                                const Acceleration &ff, Acceleration &u_bar)
  {
    // Set targets
    x_controller_.set_target(plan.x);
    y_controller_.set_target(plan.y);
    z_controller_.set_target(plan.z);
    yaw_controller_.set_target(plan.yaw);

    // Call PID controllers iff error is large enough
    if (plan.distance_xy(estimate) > cxt.auv_epsilon_xy_) {
      u_bar.x = x_controller_.calc(estimate.x, dt) + ff.x;
      u_bar.y = y_controller_.calc(estimate.y, dt) + ff.y;
    } else {
      u_bar.x = ff.x;
      u_bar.y = ff.y;
    }

    if (plan.distance_z(estimate) > cxt.auv_epsilon_z_) {
      u_bar.z = z_controller_.calc(estimate.z, dt) + ff.z;
    } else {
      u_bar.z = ff.z;
    }

    if (plan.distance_yaw(estimate) > cxt.auv_epsilon_yaw_) {
      u_bar.yaw = yaw_controller_.calc(estimate.yaw, dt) + ff.yaw;
    } else {
      u_bar.yaw = ff.yaw;
    }
  }

  double limit(const double previous, const double next, const double dt, const double rate)
  {
    double diff = std::min(std::abs(next - previous), dt * rate);
    return next - previous < 0 ? previous - diff : previous + diff;
  }

  void JerkController::calc(const BaseContext &cxt, double dt, const Pose &plan, const Pose &estimate,
                            const Acceleration &ff, Acceleration &u_bar)
  {
    // Set targets
    x_controller_.set_target(plan.x);
    y_controller_.set_target(plan.y);
    z_controller_.set_target(plan.z);
    yaw_controller_.set_target(plan.yaw);

    // Feedforward doesn't count toward the limit
    u_bar.x = x_controller_.calc(estimate.x, dt);
    u_bar.y = y_controller_.calc(estimate.y, dt);
    u_bar.z = z_controller_.calc(estimate.z, dt);
    u_bar.yaw = yaw_controller_.calc(estimate.yaw, dt);

    // Limit jerk
    u_bar.x = limit(prev_u_bar_.x, u_bar.x, dt, cxt.auv_jerk_xy_);
    u_bar.y = limit(prev_u_bar_.y, u_bar.y, dt, cxt.auv_jerk_xy_);
    u_bar.z = limit(prev_u_bar_.z, u_bar.z, dt, cxt.auv_jerk_z_);
    u_bar.yaw = limit(prev_u_bar_.yaw, u_bar.yaw, dt, cxt.auv_jerk_yaw_);

    // Save u_bar for next time
    prev_u_bar_ = u_bar;

    // Now apply the feedforward
    u_bar.add(ff);
  }

  void BestController::calc(const BaseContext &cxt, double dt, const Pose &plan, const Pose &estimate,
                            const Acceleration &ff, Acceleration &u_bar)
  {
    // Set targets
    x_controller_.set_target(plan.x);
    y_controller_.set_target(plan.y);
    z_controller_.set_target(plan.z);
    yaw_controller_.set_target(plan.yaw);

    // Call PID controllers iff error is large enough
    // Don't include feedforward
    if (plan.distance_xy(estimate) > cxt.auv_epsilon_xy_) {
      u_bar.x = x_controller_.calc(estimate.x, dt);
      u_bar.y = y_controller_.calc(estimate.y, dt);
    } else {
      u_bar.x = 0;
      u_bar.y = 0;
    }

    if (plan.distance_z(estimate) > cxt.auv_epsilon_z_) {
      u_bar.z = z_controller_.calc(estimate.z, dt);
    } else {
      u_bar.z = 0;
    }

    if (plan.distance_yaw(estimate) > cxt.auv_epsilon_yaw_) {
      u_bar.yaw = yaw_controller_.calc(estimate.yaw, dt);
    } else {
      u_bar.yaw = 0;
    }

    // Limit jerk
    u_bar.x = limit(prev_u_bar_.x, u_bar.x, dt, cxt.auv_jerk_xy_);
    u_bar.y = limit(prev_u_bar_.y, u_bar.y, dt, cxt.auv_jerk_xy_);
    u_bar.z = limit(prev_u_bar_.z, u_bar.z, dt, cxt.auv_jerk_z_);
    u_bar.yaw = limit(prev_u_bar_.yaw, u_bar.yaw, dt, cxt.auv_jerk_yaw_);

    // Save u_bar for next time
    prev_u_bar_ = u_bar;

    // Now apply the feedforward
    u_bar.add(ff);
  }

}