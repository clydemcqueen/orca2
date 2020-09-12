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

#ifndef ORCA_BASE__ROV_CONTEXT_HPP_
#define ORCA_BASE__ROV_CONTEXT_HPP_

#include <cmath>
#include <string>
#include <vector>

#include "ros2_shared/context_macros.hpp"

#include "orca_shared/model.hpp"

namespace orca_base
{

#define ROV_NODE_ALL_PARAMS \
  CXT_MACRO_MEMBER(mode, int, 0) \
  /* 0: ROV, 1: forward, 2: back, 3: left, 4: right, 5: ccw, 6: cw, 7: up, 8: down  */ \
                            \
  CXT_MACRO_MEMBER(mass, double, 9.75) \
  CXT_MACRO_MEMBER(volume, double, 0.01) \
  CXT_MACRO_MEMBER(fluid_density, double, 997) \
  /* kg/m^3, 997 for freshwater, 1029 for seawater  */ \
 \
  CXT_MACRO_MEMBER(base_frame, std::string, "base_link") \
 \
  CXT_MACRO_MEMBER(inc_pressure, double, 500) \
  /* Pressure trim increment  */ \
  CXT_MACRO_MEMBER(inc_tilt, int, 5) \
  /* Tilt increment  */ \
  CXT_MACRO_MEMBER(inc_lights, int, 20) \
  /* Lights increment  */ \
 \
  CXT_MACRO_MEMBER(input_dead_band, float, 0.05f) \
  /* Ignore small joystick inputs  */ \
  CXT_MACRO_MEMBER(xy_limit, double, 0.5) \
  /* Attenuate joystick inputs  */ \
  CXT_MACRO_MEMBER(yaw_gain, double, 0.2) \
  /* Attenuate joystick inputs  */ \
  CXT_MACRO_MEMBER(vertical_gain, double, 0.5) \
  /* Attenuate joystick inputs  */ \
  CXT_MACRO_MEMBER(thruster_accel_limit, double, 0.05) \
  /* Limit thruster acceleration, measured in effort units  */ \
 \
  CXT_MACRO_MEMBER(rov_pid_enabled, bool, true) \
  /* Turn pid controller on/off TODO remove the rov_ prefix  */   \
 \
  CXT_MACRO_MEMBER(rov_pressure_pid_kp, double, 0.00024) \
  /* ROV hold pressure pid Kp  */ \
  CXT_MACRO_MEMBER(rov_pressure_pid_ki, double, 0.0) \
  /* ROV hold pressure pid Ki  */ \
  CXT_MACRO_MEMBER(rov_pressure_pid_kd, double, 0.0) \
  /* ROV hold pressure pid Kd  */ \
 \
  CXT_MACRO_MEMBER(planner_target_z, double, -0.5) \
  /* AUV path target z position  */ \
 \
  CXT_MACRO_MEMBER(timeout_baro_ms, int, 400) \
  /* Barometer message timeout in ms  */ \
  CXT_MACRO_MEMBER(timeout_driver_ms, int, 1000) \
  /* Driver status message timeout in ms  */ \
  CXT_MACRO_MEMBER(timeout_joy_ms, int, 1000) \
  /* Joy message timeout in ms  */ \
  CXT_MACRO_MEMBER(timer_period_ms, int, 50) \
  /* Timer period in ms  */ \
/* End of list */

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)

struct ROVContext
{
  ROV_NODE_ALL_PARAMS

  // Orca model
  orca::Model model_{};
};

}  // namespace orca_base

#endif  // ORCA_BASE__ROV_CONTEXT_HPP_
