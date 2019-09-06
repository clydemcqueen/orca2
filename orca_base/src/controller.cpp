#include "orca_base/controller.hpp"

namespace orca_base
{

  Controller::Controller(const BaseContext &cxt) :
    x_controller_{false, cxt.auv_x_pid_kp_, cxt.auv_x_pid_ki_, cxt.auv_x_pid_kd_},
    y_controller_{false, cxt.auv_y_pid_kp_, cxt.auv_y_pid_ki_, cxt.auv_y_pid_kd_},
    z_controller_{false, cxt.auv_z_pid_kp_, cxt.auv_z_pid_ki_, cxt.auv_z_pid_kd_},
    yaw_controller_{true, cxt.auv_yaw_pid_kp_, cxt.auv_yaw_pid_ki_, cxt.auv_yaw_pid_kd_}
  {
  }

  void Controller::calc(double dt, const Pose &plan, const Pose &estimate, const Acceleration &ff, Acceleration &u_bar)
  {
    x_controller_.set_target(plan.x);
    y_controller_.set_target(plan.y);
    z_controller_.set_target(plan.z);
    yaw_controller_.set_target(plan.yaw);

    u_bar.x = x_controller_.calc(estimate.x, dt, ff.x);
    u_bar.y = y_controller_.calc(estimate.y, dt, ff.y);
    u_bar.z = z_controller_.calc(estimate.z, dt, ff.z);
    u_bar.yaw = yaw_controller_.calc(estimate.yaw, dt, ff.yaw);
  }

  void DeadzoneController::calc(double dt, const Pose &plan, const Pose &estimate, const Acceleration &ff,
                                Acceleration &u_bar)
  {
    x_controller_.set_target(plan.x);
    y_controller_.set_target(plan.y);
    z_controller_.set_target(plan.z);
    yaw_controller_.set_target(plan.yaw);

    // Call PID controllers iff error is large enough
    if (plan.distance_xy(estimate) > e_xy_) {
      u_bar.x = x_controller_.calc(estimate.x, dt, ff.x);
      u_bar.y = y_controller_.calc(estimate.y, dt, ff.y);
    } else {
      u_bar.x = ff.x;
      u_bar.y = ff.y;
    }

    if (plan.distance_z(estimate) > e_z_) {
      u_bar.z = z_controller_.calc(estimate.z, dt, ff.z);
    } else {
      u_bar.z = ff.z;
    }

    if (plan.distance_yaw(estimate) > e_yaw_) {
      u_bar.yaw = yaw_controller_.calc(estimate.yaw, dt, ff.yaw);
    } else {
      u_bar.yaw = ff.yaw;
    }
  }

}