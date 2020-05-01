#include "orca_base/thrusters.hpp"

#include "orca_shared/pwm.hpp"

namespace orca_base
{

  struct Thruster
  {
    std::string frame_id;   // URDF link frame id
    bool ccw;               // True if counterclockwise
    double forward_factor;
    double strafe_factor;
    double yaw_factor;
    double vertical_factor;
  };

  // Thrusters, order must match the order of the <thruster> tags in the URDF
  const std::vector<Thruster> THRUSTERS = {
    {"t200_link_front_right",    false, 1.0, 1.0,  1.0,  0.0},
    {"t200_link_front_left",     false, 1.0, -1.0, -1.0, 0.0},
    {"t200_link_rear_right",     true,  1.0, -1.0, 1.0,  0.0},
    {"t200_link_rear_left",      true,  1.0, 1.0,  -1.0, 0.0},
    {"t200_link_vertical_right", false, 0.0, 0.0,  0.0,  1.0},
    {"t200_link_vertical_left",  true,  0.0, 0.0,  0.0,  -1.0},
  };

  void efforts_to_control(const mw::Efforts &efforts, double xy_limit, orca_msgs::msg::Control &control_msg)
  {
    // Keep track of saturation for diagnostics
    bool thruster_saturated = false;

    for (const auto &i : THRUSTERS) {

      double combined_effort = efforts.forward() * i.forward_factor + efforts.strafe() * i.strafe_factor;

      // Clamp forward + strafe to xy_limit
      if (combined_effort > xy_limit) {
        combined_effort = xy_limit;
        thruster_saturated = true;
      } else if (combined_effort < -xy_limit) {
        combined_effort = -xy_limit;
        thruster_saturated = true;
      }

      combined_effort += efforts.yaw() * i.yaw_factor;

      // Clamp forward + strafe + yaw to max values
      if (combined_effort > orca::THRUST_FULL_FWD) {
        combined_effort = orca::THRUST_FULL_FWD;
        thruster_saturated = true;
      } else if (combined_effort < orca::THRUST_FULL_REV) {
        combined_effort = orca::THRUST_FULL_REV;
        thruster_saturated = true;
      }

      double vertical_effort = efforts.vertical() * i.vertical_factor;

      // Clamp vertical effort to max values
      if (vertical_effort > orca::THRUST_FULL_FWD) {
        vertical_effort = orca::THRUST_FULL_FWD;
        thruster_saturated = true;
      } else if (vertical_effort < orca::THRUST_FULL_REV) {
        vertical_effort = orca::THRUST_FULL_REV;
        thruster_saturated = true;
      }

      // Vertical effort is independent from the rest, no need to clamp
      control_msg.thruster_pwm.push_back(orca::effort_to_pwm(combined_effort + vertical_effort));
    }

    control_msg.thruster_saturated = thruster_saturated;
  }

} // namespace orca_base