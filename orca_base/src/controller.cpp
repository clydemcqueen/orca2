#include "orca_base/controller.hpp"

using namespace orca;

namespace orca_base
{

  PoseController::PoseController(const BaseContext &cxt) :
    cxt_{cxt},
    x_controller_{false, cxt.auv_x_pid_ku_, cxt.auv_x_pid_tu_},
    y_controller_{false, cxt.auv_y_pid_ku_, cxt.auv_y_pid_tu_},
    z_controller_{false, cxt.auv_z_pid_ku_, cxt.auv_z_pid_tu_},
    yaw_controller_{true, cxt.auv_yaw_pid_ku_, cxt.auv_yaw_pid_tu_}
  {}

  void PoseController::calc(double dt, const FP &plan, const FP &estimate,
                            const Acceleration &ff, orca::Efforts &efforts)
  {
#if 1
    orca::Acceleration u_bar = ff;

    // Trust x, y and yaw if the covar is low and we have a fairly close observation
    if (estimate.good_pose()) {
      x_controller_.set_target(plan.pose.pose.x);
      u_bar.x = x_controller_.calc(estimate.pose.pose.x, dt) + ff.x;

      y_controller_.set_target(plan.pose.pose.y);
      u_bar.y = y_controller_.calc(estimate.pose.pose.y, dt) + ff.y;

      yaw_controller_.set_target(plan.pose.pose.yaw);
      u_bar.yaw = yaw_controller_.calc(estimate.pose.pose.yaw, dt) + ff.yaw;
    }

    // Trust z if we're getting it from baro
    if (estimate.good_z()) {
      z_controller_.set_target(plan.pose.pose.z);
      u_bar.z = z_controller_.calc(estimate.pose.pose.z, dt) + ff.z;
    }

    efforts.from_acceleration(plan.pose.pose.yaw, u_bar);

#endif

#if 0
    u_bar = ff;

    if (estimate.pose.x_valid) {
      x_controller_.set_target(plan.x);
      u_bar.x = x_controller_.calc(estimate.pose.pose.x, dt) + ff.x;
    }

    if (estimate.pose.y_valid) {
      y_controller_.set_target(plan.y);
      u_bar.y = y_controller_.calc(estimate.pose.pose.y, dt) + ff.y;
    }

    if (estimate.pose.z_valid) {
      z_controller_.set_target(plan.z);
      u_bar.z = z_controller_.calc(estimate.pose.pose.z, dt) + ff.z;
    }

    if (estimate.pose.yaw_valid) {
      yaw_controller_.set_target(plan.yaw);
      u_bar.yaw = yaw_controller_.calc(estimate.pose.pose.yaw, dt) + ff.yaw;
    }
#endif

#if 0
    // Set targets
    x_controller_.set_target(plan.x);
    y_controller_.set_target(plan.y);
    z_controller_.set_target(plan.z);
    yaw_controller_.set_target(plan.yaw);

    // Calculate response to the error
    u_bar.x = x_controller_.calc(estimate.pose.pose.x, dt) + ff.x;
    u_bar.y = y_controller_.calc(estimate.pose.pose.y, dt) + ff.y;
    u_bar.z = z_controller_.calc(estimate.pose.pose.z, dt) + ff.z;
    u_bar.yaw = yaw_controller_.calc(get_yaw(estimate.pose.pose.orientation), dt) + ff.yaw;
#endif
  }

#if 0
  void DeadzoneController::calc(const BaseContext &cxt, double dt, const FP &plan, const FP &estimate,
                                const Acceleration &ff, Acceleration &u_bar)
  {
    // Set targets
    x_controller_.set_target(plan.pose.pose.x);
    y_controller_.set_target(plan.pose.pose.y);
    z_controller_.set_target(plan.pose.pose.z);
    yaw_controller_.set_target(plan.pose.pose.yaw);

    // Call PID controllers iff error is large enough
    if (plan.pose.pose.distance_xy(estimate.pose.pose) > cxt.auv_epsilon_xy_) {
      u_bar.x = x_controller_.calc(estimate.pose.pose.x, dt) + ff.x;
      u_bar.y = y_controller_.calc(estimate.pose.pose.y, dt) + ff.y;
    } else {
      u_bar.x = ff.x;
      u_bar.y = ff.y;
    }

    if (plan.pose.pose.distance_z(estimate.pose.pose) > cxt.auv_epsilon_z_) {
      u_bar.z = z_controller_.calc(estimate.pose.pose.z, dt) + ff.z;
    } else {
      u_bar.z = ff.z;
    }

    if (plan.pose.pose.distance_yaw(estimate.pose.pose) > cxt.auv_epsilon_yaw_) {
      u_bar.yaw = yaw_controller_.calc(estimate.pose.pose.yaw, dt) + ff.yaw;
    } else {
      u_bar.yaw = ff.yaw;
    }
  }

  double limit(const double previous, const double next, const double dt, const double rate)
  {
    double diff = std::min(std::abs(next - previous), dt * rate);
    return next - previous < 0 ? previous - diff : previous + diff;
  }

  void JerkController::calc(const BaseContext &cxt, double dt, const FP &plan, const FP &estimate,
                            const Acceleration &ff, Acceleration &u_bar)
  {
    // Set targets
    x_controller_.set_target(plan.pose.pose.x);
    y_controller_.set_target(plan.pose.pose.y);
    z_controller_.set_target(plan.pose.pose.z);
    yaw_controller_.set_target(plan.pose.pose.yaw);

    // Feedforward doesn't count toward the limit
    u_bar.x = x_controller_.calc(estimate.pose.pose.x, dt);
    u_bar.y = y_controller_.calc(estimate.pose.pose.y, dt);
    u_bar.z = z_controller_.calc(estimate.pose.pose.z, dt);
    u_bar.yaw = yaw_controller_.calc(estimate.pose.pose.yaw, dt);

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

  void BestController::calc(const BaseContext &cxt, double dt, const FP &plan, const FP &estimate,
                            const Acceleration &ff, Acceleration &u_bar)
  {
    // Set targets
    x_controller_.set_target(plan.pose.pose.x);
    y_controller_.set_target(plan.pose.pose.y);
    z_controller_.set_target(plan.pose.pose.z);
    yaw_controller_.set_target(plan.pose.pose.yaw);

    // Call PID controllers iff error is large enough
    // Don't include feedforward
    if (plan.pose.pose.distance_xy(estimate.pose.pose) > cxt.auv_epsilon_xy_) {
      u_bar.x = x_controller_.calc(estimate.pose.pose.x, dt);
      u_bar.y = y_controller_.calc(estimate.pose.pose.y, dt);
    } else {
      u_bar.x = 0;
      u_bar.y = 0;
    }

    if (plan.pose.pose.distance_z(estimate.pose.pose) > cxt.auv_epsilon_z_) {
      u_bar.z = z_controller_.calc(estimate.pose.pose.z, dt);
    } else {
      u_bar.z = 0;
    }

    if (plan.pose.pose.distance_yaw(estimate.pose.pose) > cxt.auv_epsilon_yaw_) {
      u_bar.yaw = yaw_controller_.calc(estimate.pose.pose.yaw, dt);
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

  void DepthController::calc(const BaseContext &cxt, double dt, const FP &plan, const FP &estimate,
                             const Acceleration &ff, Acceleration &u_bar)
  {
    u_bar = ff;

    z_controller_.set_target(plan.pose.pose.z);
    u_bar.z = z_controller_.calc(estimate.pose.pose.z, dt) + ff.z;
  }
#endif

  MoveToMarkerController::MoveToMarkerController(const BaseContext &cxt) :
    cxt_{cxt},
    forward_controller_{false, 0.04, 0, 0},
    vertical_controller_{false, cxt.auv_z_pid_ku_, cxt.auv_z_pid_tu_},
    yaw_controller_{true, 0.5, 0, 0}
  {}

  void MoveToMarkerController::calc(double dt, const orca::Observation &plan, double plan_z,
                                    const orca::Observation &estimate, double estimate_z,
                                    const orca::AccelerationBody &ff, orca::Efforts &efforts)
  {
    orca::AccelerationBody u_bar = ff;

    // If we have an observation, run the PID controller(s)
    if (estimate.id != orca::NOT_A_MARKER) {
      forward_controller_.set_target(plan.distance);
      yaw_controller_.set_target(plan.yaw);

      // The distance calculation is quite noisy, so just use the ff for now
      // Re-visit this if obs.destination is filtered
//    u_bar.forward = -forward_controller_.calc(estimate.distance, dt) + ff.forward;
      u_bar.forward = ff.forward;
      u_bar.yaw = yaw_controller_.calc(estimate.yaw, dt) + ff.yaw;
    }

    // Always run the vertical PID controller
    vertical_controller_.set_target(plan_z);
    u_bar.vertical = vertical_controller_.calc(estimate_z, dt) + ff.vertical;

    // Compute efforts
    efforts.set_forward(Model::accel_to_effort_xy(u_bar.forward));
    efforts.set_strafe(0);
    efforts.set_vertical(Model::accel_to_effort_z(u_bar.vertical));
    efforts.set_yaw(-Model::accel_to_effort_yaw(u_bar.yaw));
  }

}