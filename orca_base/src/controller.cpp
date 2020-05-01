#include "orca_base/controller.hpp"

#include "orca_shared/util.hpp"

namespace orca_base
{

  PoseController::PoseController(const AUVContext &cxt) :
    cxt_{cxt},
    x_controller_{false, cxt.auv_x_pid_kp_, cxt.auv_x_pid_ki_, cxt.auv_x_pid_kd_},
    y_controller_{false, cxt.auv_y_pid_kp_, cxt.auv_y_pid_ki_, cxt.auv_y_pid_kd_},
    z_controller_{false, cxt.auv_z_pid_kp_, cxt.auv_z_pid_ki_, cxt.auv_z_pid_kd_},
    yaw_controller_{true, cxt.auv_yaw_pid_kp_, cxt.auv_yaw_pid_ki_, cxt.auv_yaw_pid_kd_}
  {}

  void PoseController::calc(const rclcpp::Duration &d, const mw::FiducialPose &plan, const mw::FiducialPose &estimate,
                            const mw::Acceleration &ff, mw::Efforts &efforts)
  {
    auto dt = d.seconds();

    // Init u_bar to feedforward
    auto u_bar = ff;

    // Trust x, y and yaw if the covar is low and we have a fairly close observation
    if (estimate.good(cxt_.good_pose_dist_)) {
      x_controller_.set_target(plan.pose().pose().x());
      u_bar.x() = x_controller_.calc(estimate.pose().pose().x(), dt) + ff.x();

      y_controller_.set_target(plan.pose().pose().y());
      u_bar.y() = y_controller_.calc(estimate.pose().pose().y(), dt) + ff.y();

      yaw_controller_.set_target(plan.pose().pose().yaw());
      u_bar.yaw() = yaw_controller_.calc(estimate.pose().pose().yaw(), dt) + ff.yaw();
    }

    // Trust z if we're getting it from baro
    if (estimate.pose().good1()) {
      z_controller_.set_target(plan.pose().pose().z());
      u_bar.z() = z_controller_.calc(estimate.pose().pose().z(), dt) + ff.z();
    }

    efforts = {cxt_.model_, plan.pose().pose().yaw(), u_bar};
  }

  ObservationController::ObservationController(const AUVContext &cxt) :
    cxt_{cxt},
    forward_controller_{false, cxt.mtm_fwd_pid_kp_, cxt.mtm_fwd_pid_ki_, cxt.mtm_fwd_pid_kd_},
    vertical_controller_{false, cxt.auv_z_pid_kp_, cxt.auv_z_pid_ki_, cxt.auv_z_pid_kd_}, // Re-use auv_z
    bearing_controller_{true, cxt.auv_yaw_pid_kp_, cxt.auv_yaw_pid_ki_, cxt.auv_yaw_pid_kd_} // Re-use auv_yaw
  {}

  // Observations are in the camera_link frame, not the base_link frame
  // This means that the sub is ~20cm further away than obs.distance, and obs.bearing is exaggerated
  // In practice we can ignore this
  void ObservationController::calc(const rclcpp::Duration &d, const mw::PolarObservation &plan, double plan_z,
                                   const mw::PolarObservation &estimate, double estimate_z, const mw::AccelerationBody &ff,
                                   mw::Efforts &efforts)
  {
    auto dt = d.seconds();

    // Init u_bar to feedforward
    auto u_bar = ff;

    // If we have a fairly close observation run the PID controllers
    // Running the PID controllers from too far away causes a lot of jerk
    // TODO estimate bearing and distance uncertainty and use these to attenuate the PID controllers
    if (estimate.id() != mw::NOT_A_MARKER && estimate.distance() < 5 /* TODO param */) {
      bearing_controller_.set_target(plan.bearing());
      u_bar.yaw() = bearing_controller_.calc(estimate.bearing(), dt) + ff.yaw();

      forward_controller_.set_target(plan.distance());
      u_bar.forward() = -forward_controller_.calc(estimate.distance(), dt) + ff.forward();
    }

    // Always run the vertical PID controller
    vertical_controller_.set_target(plan_z);
    u_bar.vertical() = vertical_controller_.calc(estimate_z, dt) + ff.vertical();

    // Compute efforts
    efforts.forward(cxt_.model_.accel_to_effort_xy(u_bar.forward()));
    efforts.strafe(0);
    efforts.vertical(cxt_.model_.accel_to_effort_z(u_bar.vertical()));
    efforts.yaw(-cxt_.model_.accel_to_effort_yaw(u_bar.yaw()));
  }

}