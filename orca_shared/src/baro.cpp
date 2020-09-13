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
 * When the sub is floating at the surface of the water:
 * base_link_z ~= -0.125
 * baro_link_z ~= -0.075
 */

namespace orca
{

// TODO(clyde): get from urdf or tf tree
static const double baro_link_to_base_link_z = -0.05;

void Barometer::initialize(const Model & model, double pressure, double base_link_z)
{
  // Transform base_link to baro_link
  double baro_link_z = base_link_z - baro_link_to_base_link_z;

  // Initialize atmospheric pressure
  atmospheric_pressure_ = model.atmospheric_pressure(pressure, baro_link_z);
}

double Barometer::pressure_to_base_link_z(const Model & model, double pressure) const
{
  if (!initialized()) {
    std::cout << "barometer is not initialized" << std::endl;
    return 0;
  }

  // Calc depth from pressure
  double baro_link_z = model.pressure_to_z(atmospheric_pressure_, pressure);

  // Transform baro_link to base_link
  double base_link_z = baro_link_z + baro_link_to_base_link_z;

  return base_link_z;
}

}  // namespace orca
