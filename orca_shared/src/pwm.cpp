#include "orca_shared/pwm.hpp"

namespace orca
{

  uint16_t effort_to_pwm(const double effort)
  {
    return orca::clamp(
      static_cast<uint16_t>(orca_msgs::msg::Control::THRUST_STOP +
                            (effort > THRUST_STOP ? THRUST_DZ_PWM : (effort < THRUST_STOP ? -THRUST_DZ_PWM : 0)) +
                            std::round(effort * THRUST_RANGE_PWM)),
      orca_msgs::msg::Control::THRUST_FULL_REV,
      orca_msgs::msg::Control::THRUST_FULL_FWD);
  }

} // namespace orca_base
