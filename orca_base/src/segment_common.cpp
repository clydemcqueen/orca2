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

#include "orca_base/segment_common.hpp"

#include <iomanip>
#include <string>
#include <utility>

#include "orca_msgs/msg/control.hpp"
#include "orca_shared/util.hpp"

namespace orca_base
{

//=====================================================================================
// SegmentBase
//=====================================================================================

SegmentBase::SegmentBase(AUVContext cxt, uint8_t type)
: cxt_{std::move(cxt)}, type_{type}
{
}

std::string SegmentBase::type_name() const
{
  if (type_ == orca_msgs::msg::MissionState::PAUSE) {
    return "pause";
  } else if (type_ == orca_msgs::msg::MissionState::POSE_VERTICAL) {
    return "pose_vertical";
  } else if (type_ == orca_msgs::msg::MissionState::POSE_ROTATE) {
    return "pose_rotate";
  } else if (type_ == orca_msgs::msg::MissionState::POSE_LINE) {
    return "pose_line";
  } else if (type_ == orca_msgs::msg::MissionState::POSE_COMBO) {
    return "pose_combo";
  } else if (type_ == orca_msgs::msg::MissionState::OBS_RTM) {
    return "obs_rtm";
  } else if (type_ == orca_msgs::msg::MissionState::OBS_MTM) {
    return "obs_mtm";
  } else {
    return "no_segment";
  }
}

//=====================================================================================
// FastPlan
//=====================================================================================

FastPlan::FastPlan(bool angle, double d, double a_max, double v_max)
{
  assert(a_max > 0);
  assert(v_max > 0);

  if (angle) {
    d = orca::norm_angle(d);
  }

  d = std::abs(d);

  t_ramp = v_max / a_max;
  auto d_run = d - a_max * t_ramp * t_ramp;
  if (angle) {
    d_run = orca::norm_angle(d_run);
  }
  t_run = d_run / v_max;

  if (t_run < 0) {
    // Distance too short, will not hit v_max
    t_ramp = sqrt(d / a_max);
    t_run = 0;
  }
}

std::ostream & operator<<(std::ostream & os, FastPlan const & p)
{
  return os << std::fixed << std::setprecision(3) << "{t_ramp: " << p.t_ramp << ", t_run: " <<
         p.t_run << "}";
}

//=====================================================================================
// SyncPlan
//=====================================================================================

SyncPlan::SyncPlan(bool angle, double d, double t_ramp, double t_run)
{
  if (angle) {
    d = orca::norm_angle(d);
  }

  a = d / (t_ramp * (t_ramp + t_run));
  v = a * t_ramp;
  d_ramp = 0.5 * v * t_ramp;
  d_run = v * t_run;
}

std::ostream & operator<<(std::ostream & os, SyncPlan const & p)
{
  return os << std::fixed << std::setprecision(3) <<
         "{a: " << p.a << ", v: " << p.v << ", d_ramp: " << p.d_ramp << ", d_run: " << p.d_run <<
         "}";
}

}  // namespace orca_base
