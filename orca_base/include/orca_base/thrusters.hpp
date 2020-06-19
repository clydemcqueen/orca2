#ifndef ORCA_BASE_THRUSTERS_HPP
#define ORCA_BASE_THRUSTERS_HPP

#include "orca_msgs/msg/control.hpp"
#include "orca_shared/mw/efforts.hpp"

namespace orca_base
{

struct Thruster
{
  std::string frame_id;   // URDF link frame id
  bool ccw;               // True if counterclockwise
  double forward;
  double strafe;
  double yaw;
  double vertical;
  double prev_effort;     // Most recent effort

  Thruster(std::string _frame_id, bool _ccw,
    double _forward, double _strafe, double _yaw, double _vertical)
    :
    frame_id{std::move(_frame_id)},
    ccw{_ccw},
    forward{_forward},
    strafe{_strafe},
    yaw{_yaw},
    vertical{_vertical},
    prev_effort{} {}

  int efforts_to_pwm(const mw::Efforts & efforts, double xy_limit, double max_change,
    bool & saturated);
};

class Thrusters
{
  std::vector<Thruster> thrusters_;
  std::vector<int> prev_pwm_;

public:

  Thrusters();

  /**
   * Combine efforts (forward, strafe, vertical, yaw) to the 6 thruster PWM signals, and write to a control message
   *
   * @param efforts Efforts
   * @param xy_limit Limit forward + strafe efforts to this value
   * @param control_msg Write to this control message
   */
  void efforts_to_control(const mw::Efforts & efforts, double xy_limit, double thruster_accel_limit,
    orca_msgs::msg::Control & control_msg);
};

} // namespace orca_base

#endif // ORCA_BASE_THRUSTERS_HPP
