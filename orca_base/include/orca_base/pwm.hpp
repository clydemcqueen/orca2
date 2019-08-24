#ifndef ORCA_BASE_PWM_HPP
#define ORCA_BASE_PWM_HPP

#include <cstdint>

#include "orca_base/util.hpp"

namespace orca_base
{
  // TODO use orca_msgs::msg::Control constants

  //---------------------------------
  // Camera tilt servo
  //---------------------------------

  // Domain
  constexpr int TILT_MIN = -45;
  constexpr int TILT_MAX = 45;

  // Range
  constexpr uint16_t TILT_MIN_PWM = 1100;
  constexpr uint16_t TILT_MAX_PWM = 1900;

  constexpr uint16_t tilt_to_pwm(const int tilt)
  {
    return scale(tilt, TILT_MIN, TILT_MAX, TILT_MIN_PWM, TILT_MAX_PWM);
  }

  constexpr int pwm_to_tilt(const uint16_t pwm)
  {
    return scale(pwm, TILT_MIN_PWM, TILT_MAX_PWM, TILT_MIN, TILT_MAX);
  }

  //---------------------------------
  // BlueRobotics Lumen subsea light
  //---------------------------------

  // Domain
  constexpr int BRIGHTNESS_MIN = 0;
  constexpr int BRIGHTNESS_MAX = 100;

  // Range
  constexpr uint16_t BRIGHTNESS_MIN_PWM = 1100;
  constexpr uint16_t BRIGHTNESS_MAX_PWM = 1900;

  constexpr uint16_t brightness_to_pwm(const int brightness)
  {
    return scale(brightness, BRIGHTNESS_MIN, BRIGHTNESS_MAX, BRIGHTNESS_MIN_PWM, BRIGHTNESS_MAX_PWM);
  }

  constexpr int pwm_to_brightness(const uint16_t pwm)
  {
    return scale(pwm, BRIGHTNESS_MIN_PWM, BRIGHTNESS_MAX_PWM, BRIGHTNESS_MIN, BRIGHTNESS_MAX);
  }

  //---------------------------------
  // BlueRobotics T200 thruster + ESC
  //---------------------------------

  // Domain
  constexpr double THRUST_FULL_REV = -1.0;
  constexpr double THRUST_STOP = 0.0;
  constexpr double THRUST_FULL_FWD = 1.0;

  // Range with deadzone
  constexpr uint16_t THRUST_FULL_REV_PWM = 1100;
  constexpr uint16_t THRUST_STOP_PWM = 1500;
  constexpr uint16_t THRUST_FULL_FWD_PWM = 1900;
  constexpr uint16_t THRUST_DZ_PWM = 30;
  constexpr uint16_t THRUST_RANGE_PWM = 400 - THRUST_DZ_PWM;

  uint16_t effort_to_pwm(const double effort)
  {
    return clamp(static_cast<uint16_t>(
                   THRUST_STOP_PWM +
                   (effort > THRUST_STOP ? THRUST_DZ_PWM : (effort < THRUST_STOP ? -THRUST_DZ_PWM : 0)) +
                   std::round(effort * THRUST_RANGE_PWM)
                 ), THRUST_FULL_REV_PWM, THRUST_FULL_FWD_PWM);
  }

  constexpr double pwm_to_effort(const uint16_t pwm)
  {
    return static_cast<double>(
             pwm - THRUST_STOP_PWM +
             (pwm > THRUST_STOP_PWM ? -THRUST_DZ_PWM : (pwm < THRUST_STOP_PWM ? THRUST_DZ_PWM : 0))
           ) / THRUST_RANGE_PWM;
  }

} // namespace orca_base

#endif // ORCA_BASE_PWM_HPP
