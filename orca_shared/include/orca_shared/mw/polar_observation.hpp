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

#ifndef ORCA_SHARED__MW__POLAR_OBSERVATION_HPP_
#define ORCA_SHARED__MW__POLAR_OBSERVATION_HPP_

#include "orca_msgs/msg/polar_observation.hpp"

namespace mw
{

class PolarObservation
{
  orca_msgs::msg::PolarObservation msg_;

public:
  static const PolarObservation None;

  PolarObservation() = default;

  explicit PolarObservation(const orca_msgs::msg::PolarObservation & msg)
  : msg_{msg} {}

  PolarObservation(const int & id, const double & distance, const double & bearing)
  {
    msg_.id = id;
    msg_.distance = distance;
    msg_.bearing = bearing;
  }

  orca_msgs::msg::PolarObservation msg() const
  {
    return msg_;
  }

  int id() const
  {
    return msg_.id;
  }

  double distance() const
  {
    return msg_.distance;
  }

  double bearing() const
  {
    return msg_.bearing;
  }

  int & id()
  {
    return msg_.id;
  }

  double & distance()
  {
    return msg_.distance;
  }

  double & bearing()
  {
    return msg_.bearing;
  }

  void id(const int & v)
  {
    msg_.id = v;
  }

  void distance(const double & v)
  {
    msg_.distance = v;
  }

  void bearing(const double & v)
  {
    msg_.bearing = v;
  }

  bool operator==(const PolarObservation & that) const
  {
    return msg_ == that.msg_;
  }

  bool operator!=(const PolarObservation & that) const
  {
    return !(*this == that);
  }

  friend std::ostream & operator<<(std::ostream & os, PolarObservation const & v);
};

}  // namespace mw

#endif  // ORCA_SHARED__MW__POLAR_OBSERVATION_HPP_
