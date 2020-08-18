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

#ifndef ORCA_GAZEBO__ORCA_GAZEBO_UTIL_HPP_
#define ORCA_GAZEBO__ORCA_GAZEBO_UTIL_HPP_

#include "ignition/math/Vector3.hh"
#include "ignition/math/Quaternion.hh"

#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace orca_gazebo
{
std::default_random_engine k_generator;

// Assume x, y and z are independent, e.g., MEMS accelerometers or gyros
void addNoise(const double stddev, ignition::math::Vector3d & v)
{
  std::normal_distribution<double> distribution(0, stddev);

  v.X() += distribution(k_generator);
  v.Y() += distribution(k_generator);
  v.Z() += distribution(k_generator);
}

// Assume r, p and y are independent, e.g., MEMS magnetometers
void addNoise(const double stddev, ignition::math::Quaterniond & q)
{
  ignition::math::Vector3d v = q.Euler();
  addNoise(stddev, v);
  q.Euler(v);
}

void ignition2msg(const ignition::math::Vector3d & i, geometry_msgs::msg::Vector3 & m)
{
  m.x = i.X();
  m.y = i.Y();
  m.z = i.Z();
}

void ignition2msg(const ignition::math::Quaterniond & i, geometry_msgs::msg::Quaternion & m)
{
  m.x = i.X();
  m.y = i.Y();
  m.z = i.Z();
  m.w = i.W();
}

}  // namespace orca_gazebo

#endif  // ORCA_GAZEBO__ORCA_GAZEBO_UTIL_HPP_
