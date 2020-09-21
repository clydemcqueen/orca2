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

#include <cmath>
#include <iostream>

#include "orca_base/pid.hpp"

namespace pid
{

Controller::Controller(bool angle, double Kp, double Ki, double Kd)
{
  angle_ = angle;
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
}

void Controller::set_target(double target)
{
  if (angle_) {
    while (target < -M_PI) {
      target += 2 * M_PI;
    }
    while (target > M_PI) {
      target -= 2 * M_PI;
    }
  }

  if (std::abs(target - target_) > 0.001) {
    // std::cout << "old target: " << target_ << ", new target: " << target << std::endl;
    // std::cout << "prev_error: " << prev_error_ << ", integral: " << integral_ << std::endl;

    target_ = target;
    prev_error_ = 0;
    integral_ = 0;
  }
}

// Run one calculation
double Controller::calc(double state, double dt)
{
  double error = target_ - state;

  if (angle_) {
    while (error < -M_PI) {
      error += 2 * M_PI;

      // Derivative and integral are poorly defined at the discontinuity
      prev_error_ = 0;
      integral_ = 0;
    }
    while (error > M_PI) {
      error -= 2 * M_PI;

      prev_error_ = 0;
      integral_ = 0;
    }
  }

  integral_ = integral_ + (error * dt);
  double derivative = (error - prev_error_) / dt;
  prev_error_ = error;

  return Kp_ * error + Ki_ * integral_ + Kd_ * derivative;
}

}  // namespace pid
