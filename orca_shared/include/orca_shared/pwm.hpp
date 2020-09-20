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

#ifndef ORCA_SHARED__PWM_HPP_
#define ORCA_SHARED__PWM_HPP_

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
  return orca::scale(pwm, orca_msgs::msg::Control::TILT_45_UP,
           orca_msgs::msg::Control::TILT_45_DOWN,
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
// TODO add deadzone
constexpr double THRUST_FULL_REV = -1.0;
constexpr double THRUST_STOP = 0.0;
constexpr double THRUST_FULL_FWD = 1.0;

uint16_t effort_to_pwm(uint16_t thrust_dz_pwm, double effort);

double pwm_to_effort(uint16_t thrust_dz_pwm, uint16_t pwm);

}  // namespace orca

#endif  // ORCA_SHARED__PWM_HPP_
