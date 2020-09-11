// Copyright (c) 2020, Clyde McQueen.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "orca_base/controller.hpp"
#include "orca_shared/util.hpp"

namespace orca_base
{

PoseController::PoseController(const AUVContext & cxt)
: cxt_{cxt},
  x_controller_{false, cxt.auv_x_pid_kp_, cxt.auv_x_pid_ki_, cxt.auv_x_pid_kd_},
  y_controller_{false, cxt.auv_y_pid_kp_, cxt.auv_y_pid_ki_, cxt.auv_y_pid_kd_},
  z_controller_{false, cxt.auv_z_pid_kp_, cxt.auv_z_pid_ki_, cxt.auv_z_pid_kd_},
  yaw_controller_{true, cxt.auv_yaw_pid_kp_, cxt.auv_yaw_pid_ki_, cxt.auv_yaw_pid_kd_} {}

void PoseController::calc(
  const rclcpp::Duration & d, const mw::FiducialPose & plan,
  const mw::FiducialPose & estimate,
  const mw::Acceleration & ff, mw::Efforts & efforts)
{
  auto dt = d.seconds();

  // Init u_bar to feedforward
  auto u_bar = ff;

  // Trust x, y and yaw if the covar is low and we have a fairly close observation
  if (cxt_.auv_pid_enabled_ && estimate.good(cxt_.good_pose_dist_)) {
    x_controller_.set_target(plan.pose().pose().x());
    u_bar.x() = x_controller_.calc(estimate.pose().pose().x(), dt) + ff.x();

    y_controller_.set_target(plan.pose().pose().y());
    u_bar.y() = y_controller_.calc(estimate.pose().pose().y(), dt) + ff.y();

    yaw_controller_.set_target(plan.pose().pose().yaw());
    u_bar.yaw() = yaw_controller_.calc(estimate.pose().pose().yaw(), dt) + ff.yaw();
  }

  // Trust z if we're getting it from baro
  if (cxt_.auv_pid_enabled_ && estimate.pose().good1()) {
    z_controller_.set_target(plan.pose().pose().z());
    u_bar.z() = z_controller_.calc(estimate.pose().pose().z(), dt) + ff.z();
  }

  // std::cout << "u_bar accel: " << u_bar << std::endl;

  // Calc efforts. Transformation from world frame to body frame requires a reasonable yaw.
  // The planned yaw is typically the best choice, but for PID tuning there's no plan, so
  // the planned yaw jumps to the final position: not reasonable.
  efforts = {cxt_.model_,
    cxt_.control_use_est_yaw_ ? estimate.pose().pose().yaw() : plan.pose().pose().yaw(), u_bar};

  // std::cout << "effort: " << efforts << std::endl;
}

ObservationController::ObservationController(const AUVContext & cxt)
: cxt_{cxt},
  forward_controller_{false, cxt.mtm_fwd_pid_kp_, cxt.mtm_fwd_pid_ki_, cxt.mtm_fwd_pid_kd_},
  vertical_controller_{false, cxt.auv_z_pid_kp_, cxt.auv_z_pid_ki_,
    cxt.auv_z_pid_kd_},  // Re-use auv_z
  bearing_controller_{true, cxt.auv_yaw_pid_kp_, cxt.auv_yaw_pid_ki_,
    cxt.auv_yaw_pid_kd_}  // Re-use auv_yaw
{}

// Observations are in the camera_link frame, not the base_link frame
// This means that the sub is ~20cm further away than obs.distance, and obs.bearing is exaggerated
// In practice we can ignore this
void ObservationController::calc(
  const rclcpp::Duration & d, const mw::PolarObservation & plan,
  double plan_z,
  const mw::PolarObservation & estimate, double estimate_z, const mw::AccelerationBody & ff,
  mw::Efforts & efforts)
{
  auto dt = d.seconds();

  // Init u_bar to feedforward
  auto u_bar = ff;

  // If we have a fairly close observation run the PID controllers
  // Running the PID controllers from too far away causes a lot of jerk
  // TODO(clyde): estimate bearing and distance uncertainty, use to attenuate PID controllers
  // TODO(clyde): 5m should be a parameter
  if (cxt_.auv_pid_enabled_ && estimate.id() != mw::NOT_A_MARKER && estimate.distance() < 5) {
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

}  // namespace orca_base
