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

#include "orca_shared/baro.hpp"

#include <iostream>

/***************************************************************
 * How is orca::Barometer initialized?
 *
 * Start the system (orca_driver, auv_node) while the sub is in the air
 * orca_driver publishes raw pressure (no adjustments) on /barometer
 * depth_node subscribes to /barometer, passes the first pressure reading to orca::Barometer
 * orca::Barometer saves the 1st reading as atmospheric_pressure_
 *
 * What about simulation?
 *
 * For 2s BarometerPlugin sends ATMOSPHERIC_PRESSURE
 * depth_node passes the first reading to orca::Barometer, etc.
 * The model should be injected into the simulation at {0, 0, 0}
 * See notes in orca_gazebo::BarometerPlugin for more information
 *
 * When the sub is floating at the surface of the water:
 * base_link_z ~= -0.125
 * baro_link_z ~= -0.075
 */

namespace orca
{

// TODO(clyde): get from urdf or tf tree
static const double baro_link_to_base_link_z = -0.05;

double Barometer::pressure_to_base_link_z(const Model & model, double pressure)
{
  // First reading is atmospheric pressure
  if (!initialized_) {
    atmospheric_pressure_ = pressure;
    initialized_ = true;
    std::cout << "atmospheric pressure is " << atmospheric_pressure_ << std::endl;
  }

  // Calc depth from pressure
  double baro_link_z = model.pressure_to_z(atmospheric_pressure_, pressure);

  // Transform baro_link to base_link
  double base_link_z = baro_link_z + baro_link_to_base_link_z;

  return base_link_z;
}

}  // namespace orca
