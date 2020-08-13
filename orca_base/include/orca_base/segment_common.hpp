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

#ifndef ORCA_BASE__SEGMENT_COMMON_HPP_
#define ORCA_BASE__SEGMENT_COMMON_HPP_

#include <string>

#include "orca_base/auv_context.hpp"
#include "rclcpp/duration.hpp"

namespace orca_base
{

//=====================================================================================
// Segments describe a trajectory from start to goal over time
//=====================================================================================

class SegmentBase
{
protected:
  AUVContext cxt_;
  uint8_t type_;

public:
  SegmentBase(AUVContext cxt, uint8_t type);

  // Return a string suitable for logging
  virtual std::string to_str() = 0;

  // Advance the motion plan by dt seconds, return true to continue, false if we're done
  virtual bool advance(const rclcpp::Duration & d) = 0;

  __uint8_t type() {return type_;}

  std::string type_name() const;
};

//=====================================================================================
// Trapezoidal velocity motion has 3 phases:
//
// Phase 1: constant acceleration from p0 to p1
// Phase 2: constant velocity from p1 to p2
// Phase 3: constant deceleration from p2 to p3
//
// If the distance is short, the constant velocity phase might be skipped
// Assumes start velocity (v0) is {0, 0, 0, 0}
//=====================================================================================

struct FastPlan
{
  double t_ramp;    // Time (seconds) for ramp up (phase 1) and ramp down (phase 3)
  double t_run;     // Time (seconds) for run (phase 2)

  /**
   * Find the fastest trapezoidal velocity plan, given max acceleration and velocity
   * Total time = 2 * t_ramp + t_run
   *
   * @param angle True if d is an angle
   * @param d The distance to cover
   * @param a_max Constraint: maximum acceleration
   * @param v_max Constraint: maximum velocity
   */
  FastPlan(bool angle, double d, double a_max, double v_max);
};

std::ostream & operator<<(std::ostream & os, FastPlan const & p);

struct SyncPlan
{
  double a;         // Acceleration for ramp
  double v;         // Velocity for run
  double d_ramp;    // Distance covered during ramp
  double d_run;     // Distance covered during run

  /**
   * Find a trapezoidal velocity plan with a given t_ramp and t_run
   *
   * @param angle True if d is an angle
   * @param d The distance to cover
   * @param t_ramp Ramp time
   * @param t_run Run time
   */
  SyncPlan(bool angle, double d, double t_ramp, double t_run);
};

std::ostream & operator<<(std::ostream & os, SyncPlan const & p);

}  // namespace orca_base

#endif  // ORCA_BASE__SEGMENT_COMMON_HPP_
