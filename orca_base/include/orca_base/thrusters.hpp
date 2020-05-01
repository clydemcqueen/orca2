#ifndef ORCA_BASE_THRUSTERS_HPP
#define ORCA_BASE_THRUSTERS_HPP

#include "orca_msgs/msg/control.hpp"
#include "orca_shared/mw/efforts.hpp"

namespace orca_base
{

  /**
   * Combine efforts (forward, strafe, vertical, yaw) to the 6 thruster PWM signals, and write to a control message
   *
   * @param efforts Efforts
   * @param xy_limit Limit forward + strafe efforts to this value
   * @param control_msg Write to this control message
   */
  void efforts_to_control(const mw::Efforts &efforts, double xy_limit, orca_msgs::msg::Control &control_msg);

} // namespace orca_base

#endif // ORCA_BASE_THRUSTERS_HPP
