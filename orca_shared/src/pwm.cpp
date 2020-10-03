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

#include "orca_shared/pwm.hpp"

namespace orca
{

// Range with deadzone
// ESC R2 has a deadzone of 25 microseconds, R3 has no deadzone
// Update after ft3 experiments: R3 does have a deadzone!
// Move this to a parameter to make experimentation easier
// constexpr uint16_t THRUST_DZ_PWM = 35;
// constexpr uint16_t THRUST_RANGE_PWM = 400 - THRUST_DZ_PWM;

uint16_t effort_to_pwm(const uint16_t thrust_dz_pwm, const double effort)
{
  uint16_t thrust_range_pwm = 400 - thrust_dz_pwm;

  return orca::clamp(
    static_cast<uint16_t>(orca_msgs::msg::Control::THRUST_STOP +
    (effort > THRUST_STOP ? thrust_dz_pwm : (effort < THRUST_STOP ?
    -thrust_dz_pwm : 0)) +
    std::round(effort * thrust_range_pwm)),
    orca_msgs::msg::Control::THRUST_FULL_REV,
    orca_msgs::msg::Control::THRUST_FULL_FWD);
}

double pwm_to_effort(const uint16_t thrust_dz_pwm, const uint16_t pwm)
{
  uint16_t thrust_range_pwm = 400 - thrust_dz_pwm;

  return static_cast<double>(
    pwm - orca_msgs::msg::Control::THRUST_STOP +
    (pwm > orca_msgs::msg::Control::THRUST_STOP ? -thrust_dz_pwm :
    (pwm < orca_msgs::msg::Control::THRUST_STOP ? thrust_dz_pwm : 0))) /
         thrust_range_pwm;
}

}  // namespace orca
