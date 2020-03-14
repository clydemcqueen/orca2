#ifndef ORCA_BASE_PWM_HPP
#define ORCA_BASE_PWM_HPP

#include <cstdint>

#include "orca_shared/util.hpp"

#include "orca_msgs/msg/control.hpp"

namespace orca
{
  //---------------------------------
  // Camera tilt servo
  //---------------------------------

  // Domain
  constexpr int TILT_MIN = -45;
  constexpr int TILT_MAX = 45;

  constexpr uint16_t tilt_to_pwm(const int tilt)
  {
    return orca::scale(tilt, TILT_MIN, TILT_MAX,
                       orca_msgs::msg::Control::TILT_45_UP, orca_msgs::msg::Control::TILT_45_DOWN);
  }

  constexpr int pwm_to_tilt(const uint16_t pwm)
  {
    return orca::scale(pwm, orca_msgs::msg::Control::TILT_45_UP, orca_msgs::msg::Control::TILT_45_DOWN,
                       TILT_MIN, TILT_MAX);
  }

  //---------------------------------
  // BlueRobotics Lumen subsea light
  //---------------------------------

  // Domain
  constexpr int BRIGHTNESS_MIN = 0;
  constexpr int BRIGHTNESS_MAX = 100;

  constexpr uint16_t brightness_to_pwm(const int brightness)
  {
    return orca::scale(brightness, BRIGHTNESS_MIN, BRIGHTNESS_MAX,
                       orca_msgs::msg::Control::LIGHTS_OFF, orca_msgs::msg::Control::LIGHTS_FULL);
  }

  constexpr int pwm_to_brightness(const uint16_t pwm)
  {
    return orca::scale(pwm, orca_msgs::msg::Control::LIGHTS_OFF, orca_msgs::msg::Control::LIGHTS_FULL,
                       BRIGHTNESS_MIN, BRIGHTNESS_MAX);
  }

  //---------------------------------
  // BlueRobotics T200 thruster + ESC
  //---------------------------------

  // Domain
  constexpr double THRUST_FULL_REV = -1.0;
  constexpr double THRUST_STOP = 0.0;
  constexpr double THRUST_FULL_FWD = 1.0;

  // Range with deadzone
  constexpr uint16_t THRUST_DZ_PWM = 0;  // ESC R2 has a deadzone of 25 microseconds, R3 has no deadzone
  constexpr uint16_t THRUST_RANGE_PWM = 400 - THRUST_DZ_PWM;

  uint16_t effort_to_pwm(const double effort);

  constexpr double pwm_to_effort(const uint16_t pwm)
  {
    return static_cast<double>(
             pwm - orca_msgs::msg::Control::THRUST_STOP +
             (pwm > orca_msgs::msg::Control::THRUST_STOP ? -THRUST_DZ_PWM :
              (pwm < orca_msgs::msg::Control::THRUST_STOP ? THRUST_DZ_PWM : 0))) / THRUST_RANGE_PWM;
  }

} // namespace orca_base

#endif // ORCA_BASE_PWM_HPP
