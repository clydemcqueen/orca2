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

#ifndef ORCA_FILTER__POSE_FILTER_CONTEXT_HPP_
#define ORCA_FILTER__POSE_FILTER_CONTEXT_HPP_

#include <cmath>
#include <string>
#include <vector>

#include "orca_shared/model.hpp"

namespace orca_filter
{

#define POSE_FILTER_NODE_PARAMS \
  CXT_MACRO_MEMBER(fluid_density, double, 997) \
  /* kg/m^3, 997 for freshwater, 1029 for seawater  */ \
 \
  CXT_MACRO_MEMBER(timeout_open_water_ms, int, 300) \
  /* Flip to a depth filter when we hit open water  */ \
  CXT_MACRO_MEMBER(timeout_outlier_ms, int, 300) \
  /* Reset the filter if poses are consistently rejected as outliers  */ \
 \
  CXT_MACRO_MEMBER(frame_id_map, std::string, "map") \
  CXT_MACRO_MEMBER(frame_id_base_link, std::string, "base_link") \
 \
  CXT_MACRO_MEMBER(publish_tf, bool, true) \
  /* Publish t_map_base  */ \
 \
  CXT_MACRO_MEMBER(predict_accel, bool, true) \
  /* Predict acceleration  */ \
  CXT_MACRO_MEMBER(predict_accel_control, bool, true) \
  /* Add u_bar to predicted acceleration  */ \
  CXT_MACRO_MEMBER(predict_accel_drag, bool, true) \
  /* Add drag to predicted acceleration  */ \
  CXT_MACRO_MEMBER(predict_accel_buoyancy, bool, true) \
  /* Add gravity and buoyancy to predicted acceleration  */ \
 \
  CXT_MACRO_MEMBER(filter_baro, bool, true) \
  /* Filter barometer messages  */ \
  CXT_MACRO_MEMBER(filter_fcam, bool, false) \
  /* Filter forward camera messages  */ \
 \
  CXT_MACRO_MEMBER(good_pose_dist, double, 1.8) \
  /* Good pose if marker < 1.8m away  */ \
  CXT_MACRO_MEMBER(good_obs_dist, double, 10) \
  /* Good observation if marker < 10m away  */ \
 \
  CXT_MACRO_MEMBER(four_dof, bool, false) \
  /* Experiment: run 4dof filter instead of 6dof filter  */ \
  CXT_MACRO_MEMBER(min_dt, double, 0.001) \
  /* Minimum dt between message timestamps  */ \
  CXT_MACRO_MEMBER(max_dt, double, 0.3) \
  /* Maxiumum dt between message timestamps  */ \
  CXT_MACRO_MEMBER(default_dt, double, 0.1) \
  /* If dt > max, use default instead  */ \
  CXT_MACRO_MEMBER(always_publish_odom, bool, false) \
  /* False: don't publish odom from depth messages in a pose filter  */ \
 \
  CXT_MACRO_MEMBER(ukf_alpha, double, 0.001) \
  /* UKF alpha -- see UKF  */ \
  CXT_MACRO_MEMBER(ukf_beta, double, 2.0) \
  /* UKF beta -- see UKF  */ \
  CXT_MACRO_MEMBER(ukf_kappa, int, 0) \
  /* UKF kappa -- see UKF  */ \
  CXT_MACRO_MEMBER(ukf_process_noise, double, 0.01) \
  /* UKF process noise, all dimensions TODO probably need to split dof  */ \
  CXT_MACRO_MEMBER(ukf_outlier_distance, double, 4.0) \
  /* Reject measurements > n std devs from estimate  */ \
/* End of list */

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)

struct PoseFilterContext : orca::Model
{
  POSE_FILTER_NODE_PARAMS
};

#define POSE_FILTER_NODE_ALL_PARAMS \
  MODEL_PARAMS \
  POSE_FILTER_NODE_PARAMS \
/* End of list */

}  // namespace orca_filter

#endif  // ORCA_FILTER__POSE_FILTER_CONTEXT_HPP_
