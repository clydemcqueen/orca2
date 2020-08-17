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

#ifndef ORCA_DRIVER__DRIVER_CONTEXT_HPP_
#define ORCA_DRIVER__DRIVER_CONTEXT_HPP_

#include <cmath>
#include <string>

#include "ros2_shared/context_macros.hpp"

namespace orca_driver
{

// See BlueROV2 thruster diagram: https://bluerobotics.com/learn/bluerov2-assembly/

#define DRIVER_NODE_ALL_PARAMS \
  CXT_MACRO_MEMBER(timer_period_ms, int, 100)                     /* Timer period in ms  */ \
  CXT_MACRO_MEMBER(timeout_control_ms, int, 1000)                 /* Control msg timeout */ \
 \
  CXT_MACRO_MEMBER(thruster_1_channel, int, 4)                    /* Front right */ \
  CXT_MACRO_MEMBER(thruster_2_channel, int, 7)                    /* Front left */ \
  CXT_MACRO_MEMBER(thruster_3_channel, int, 0)                    /* Rear right */ \
  CXT_MACRO_MEMBER(thruster_4_channel, int, 5)                    /* Rear left */ \
  CXT_MACRO_MEMBER(thruster_5_channel, int, 1)                    /* Vertical right */ \
  CXT_MACRO_MEMBER(thruster_6_channel, int, 6)                    /* Vertical left */ \
 \
  CXT_MACRO_MEMBER(thruster_1_reverse, bool, false) \
  CXT_MACRO_MEMBER(thruster_2_reverse, bool, false) \
  CXT_MACRO_MEMBER(thruster_3_reverse, bool, false) \
  CXT_MACRO_MEMBER(thruster_4_reverse, bool, false) \
  CXT_MACRO_MEMBER(thruster_5_reverse, bool, false) \
  CXT_MACRO_MEMBER(thruster_6_reverse, bool, false) \
 \
  CXT_MACRO_MEMBER(lights_channel, int, 9)                        /* PWM lights channel */ \
  CXT_MACRO_MEMBER(tilt_channel, int, 10)                         /* PWM tilt channel */ \
  CXT_MACRO_MEMBER(voltage_channel, int, 11)                      /* Analog voltage channel */ \
  CXT_MACRO_MEMBER(leak_channel, int, 12)                         /* Digital leak channel */ \
  CXT_MACRO_MEMBER(maestro_port, std::string, "/dev/ttyACM0")     /* Default Maestro port */ \
  CXT_MACRO_MEMBER(voltage_multiplier, double, 5.52)              /* Voltage multiplier */ \
  CXT_MACRO_MEMBER(voltage_min, double, 12.0)                     /* Minimum voltage to run  */ \
 \
  CXT_MACRO_MEMBER(read_battery, bool, true)                      /* Read voltage sensor  */ \
  CXT_MACRO_MEMBER(read_leak, bool, true)                         /* Read leak sensor  */ \
  CXT_MACRO_MEMBER(read_temp, bool, true)                         /* Read temp file  */ \
/* End of list */

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)

struct DriverContext
{
  DRIVER_NODE_ALL_PARAMS
};

}  // namespace orca_driver

#endif  // ORCA_DRIVER__DRIVER_CONTEXT_HPP_
