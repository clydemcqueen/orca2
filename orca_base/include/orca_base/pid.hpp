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

#ifndef ORCA_BASE__PID_HPP_
#define ORCA_BASE__PID_HPP_

#include <cmath>

namespace pid
{

constexpr void norm_angle(double & a)
{
  while (a < -M_PI) {
    a += 2 * M_PI;
  }
  while (a > M_PI) {
    a -= 2 * M_PI;
  }
}

class Controller
{
  bool angle_;  // True if we're controlling an angle [-pi, pi]
  double target_ = 0;
  double prev_error_ = 0;
  double integral_ = 0;
  double Kp_;
  double Ki_;
  double Kd_;

public:
  Controller(bool angle, double Kp, double Ki, double Kd)
  {
    angle_ = angle;
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
  }

  void set_target(double target)
  {
    if (angle_) {
      norm_angle(target);
    }

    if (std::abs(target - target_) > 0.001) {
      // std::cout << "set target, from " << target_ << " to " << target << std::endl;
      target_ = target;
      // prev_error_ = 0;
      // integral_ = 0;
    }
  }

  // Run one calculation
  double calc(double state, double dt)
  {
    double error = target_ - state;

    if (angle_) {
      norm_angle(error);
    }

    integral_ = integral_ + (error * dt);
    double derivative = (error - prev_error_) / dt;
    prev_error_ = error;

    return Kp_ * error + Ki_ * integral_ + Kd_ * derivative;
  }

  double target() const {return target_;}
};

}  // namespace pid

#endif  // ORCA_BASE__PID_HPP_
