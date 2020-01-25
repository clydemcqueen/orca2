#include "orca_base/controller.hpp"

using namespace orca;

namespace orca_base
{

  // TODO all controllers need to look at uncertainty and obs.distance

  void SimpleController::calc(const BaseContext &cxt, double dt, const FP &plan, const FP &estimate,
                              const Acceleration &ff, Acceleration &u_bar)
  {
#if 1
    u_bar = ff;

    if (estimate.closest_obs() < 1.8) {
      // Pose is probably good -- run PID controllers on x, y, yaw

      x_controller_.set_target(plan.pose.pose.x);
      u_bar.x = x_controller_.calc(estimate.pose.pose.x, dt) + ff.x;

      y_controller_.set_target(plan.pose.pose.y);
      u_bar.y = y_controller_.calc(estimate.pose.pose.y, dt) + ff.y;

      yaw_controller_.set_target(plan.pose.pose.yaw);
      u_bar.yaw = yaw_controller_.calc(estimate.pose.pose.yaw, dt) + ff.yaw;
    }

    z_controller_.set_target(plan.pose.pose.z);
    u_bar.z = z_controller_.calc(estimate.pose.pose.z, dt) + ff.z;
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

  void MoveToMarkerController::calc(double dt, const FP &plan, const FP &estimate,
                                    const orca::Acceleration &ff, orca::Acceleration &u_bar)
  {
    forward_controller_.set_target(plan.observations[0].distance);
    yaw_controller_.set_target(plan.observations[0].yaw);
    vertical_controller_.set_target(0.5); // TODO

    u_bar = ff;

    u_bar.x = -forward_controller_.calc(estimate.observations[0].distance, dt) + ff.x;
    u_bar.yaw = -yaw_controller_.calc(estimate.observations[0].yaw, dt) + ff.yaw;
    // TODO u_bar.z
  }

}