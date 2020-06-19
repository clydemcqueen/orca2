#include "orca_base/thrusters.hpp"

#include "orca_shared/pwm.hpp"

namespace orca_base
{

int Thruster::efforts_to_pwm(const mw::Efforts & efforts, double xy_limit,
  double thruster_accel_limit, bool & saturated)
{
  double combined_effort = efforts.forward() * forward + efforts.strafe() * strafe;

  // Clamp forward + strafe to xy_limit
  if (combined_effort > xy_limit) {
    combined_effort = xy_limit;
    saturated = true;
  } else if (combined_effort < -xy_limit) {
    combined_effort = -xy_limit;
    saturated = true;
  }

  combined_effort += efforts.yaw() * yaw;

  // Clamp forward + strafe + yaw to max values
  if (combined_effort > orca::THRUST_FULL_FWD) {
    combined_effort = orca::THRUST_FULL_FWD;
    saturated = true;
  } else if (combined_effort < orca::THRUST_FULL_REV) {
    combined_effort = orca::THRUST_FULL_REV;
    saturated = true;
  }

  double vertical_effort = efforts.vertical() * vertical;

  // Clamp vertical effort to max values
  if (vertical_effort > orca::THRUST_FULL_FWD) {
    vertical_effort = orca::THRUST_FULL_FWD;
    saturated = true;
  } else if (vertical_effort < orca::THRUST_FULL_REV) {
    vertical_effort = orca::THRUST_FULL_REV;
    saturated = true;
  }

  // Vertical effort is independent from the rest, no need to clamp
  double effort = combined_effort + vertical_effort;

  // Protect the thruster:
  // -- limit change to +/- max_change
  // -- don't reverse thruster, i.e., stop at 0.0
  effort = orca::clamp(effort,
    prev_effort - thruster_accel_limit,
    prev_effort + thruster_accel_limit);
  if (effort < 0 && prev_effort > 0 || effort > 0 && prev_effort < 0) {
    effort = 0;
  }

  prev_effort = effort;

  return orca::effort_to_pwm(effort);
}

Thrusters::Thrusters()
{
  thrusters_.emplace_back("t200_link_front_right", false, 1.0, 1.0, 1.0, 0.0);
  thrusters_.emplace_back("t200_link_front_left", false, 1.0, -1.0, -1.0, 0.0);
  thrusters_.emplace_back("t200_link_rear_right", true, 1.0, -1.0, 1.0, 0.0);
  thrusters_.emplace_back("t200_link_rear_left", true, 1.0, 1.0, -1.0, 0.0);
  thrusters_.emplace_back("t200_link_vertical_right", false, 0.0, 0.0, 0.0, 1.0);
  thrusters_.emplace_back("t200_link_vertical_left", true, 0.0, 0.0, 0.0, -1.0);

  // Thrusters and Maestro boot at 1500 (off)
  for (int i = 0; i < 6; ++i) {
    prev_pwm_.push_back(1500);
  }
}

void Thrusters::efforts_to_control(const mw::Efforts & efforts, double xy_limit,
  double thruster_accel_limit, orca_msgs::msg::Control & control_msg)
{
  // Keep track of saturation for diagnostics
  bool saturated = false;

  for (auto & i : thrusters_) {
    control_msg.thruster_pwm.push_back(
      i.efforts_to_pwm(efforts, xy_limit, thruster_accel_limit, saturated));
  }

  control_msg.thruster_saturated = saturated;
}

} // namespace orca_base