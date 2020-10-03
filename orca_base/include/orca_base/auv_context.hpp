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

#ifndef ORCA_BASE__AUV_CONTEXT_HPP_
#define ORCA_BASE__AUV_CONTEXT_HPP_

#include <cmath>
#include <string>
#include <vector>

#include "orca_base/base_context.hpp"

namespace orca_base
{

#define AUV_NODE_PARAMS \
  CXT_MACRO_MEMBER(loop_driver, int, 0) \
  /* What drives auv_advance? 0: timer, 1: depth msg, 2: fiducial msg  */ \
  CXT_MACRO_MEMBER(depth_override, bool, false) \
  /* Depth value overrides pose.position.z  */ \
  CXT_MACRO_MEMBER(timer_period_ms, int, 50) \
  /* Timer period in ms  */ \
  CXT_MACRO_MEMBER(timeout_depth_ms, int, 300) \
  /* Depth message timeout in ms  */ \
  CXT_MACRO_MEMBER(timeout_driver_ms, int, 1000) \
  /* Driver status message timeout in ms  */ \
  CXT_MACRO_MEMBER(timeout_fp_ms, int, 300) \
  /* Fiducial pose message timeout in ms  */ \
  CXT_MACRO_MEMBER(keep_poses, int, 500) \
  /* Max # of poses on filtered_path  */ \
 \
  CXT_MACRO_MEMBER(map_frame, std::string, "map") \
 \
  CXT_MACRO_MEMBER(auv_pid_enabled, bool, true) \
  /* Turn pid controllers on/off  */ \
 \
  CXT_MACRO_MEMBER(auv_x_pid_kp, double, 0.5) \
  CXT_MACRO_MEMBER(auv_x_pid_ki, double, 0.0) \
  CXT_MACRO_MEMBER(auv_x_pid_kd, double, 0.0) \
  CXT_MACRO_MEMBER(auv_x_pid_i_max, double, 0.1) \
  /* Windup prevention: max acceleration from i term (m/s^2)  */ \
 \
  CXT_MACRO_MEMBER(auv_y_pid_kp, double, 0.9) \
  CXT_MACRO_MEMBER(auv_y_pid_ki, double, 0.0) \
  CXT_MACRO_MEMBER(auv_y_pid_kd, double, 0.0) \
  CXT_MACRO_MEMBER(auv_y_pid_i_max, double, 0.1) \
 \
  CXT_MACRO_MEMBER(auv_z_pid_kp, double, 0.5) \
  CXT_MACRO_MEMBER(auv_z_pid_ki, double, 0.0) \
  CXT_MACRO_MEMBER(auv_z_pid_kd, double, 0.0) \
  CXT_MACRO_MEMBER(auv_z_pid_i_max, double, 0.1) \
 \
  CXT_MACRO_MEMBER(auv_yaw_pid_kp, double, 2.5) \
  CXT_MACRO_MEMBER(auv_yaw_pid_ki, double, 0.0) \
  CXT_MACRO_MEMBER(auv_yaw_pid_kd, double, 0.0) \
  CXT_MACRO_MEMBER(auv_yaw_pid_i_max, double, 0.1) \
  /* Windup prevention: max acceleration from i term (r/s^2)  */ \
 \
  CXT_MACRO_MEMBER(mtm_fwd_pid_kp, double, 0.4) \
  CXT_MACRO_MEMBER(mtm_fwd_pid_ki, double, 0.0) \
  CXT_MACRO_MEMBER(mtm_fwd_pid_kd, double, 0.0) \
  CXT_MACRO_MEMBER(mtm_fwd_pid_i_max, double, 0.1) \
 \
  CXT_MACRO_MEMBER(auv_xy_accel, double, 0.2) \
  /* AUV acceleration in the xy plane  */ \
  CXT_MACRO_MEMBER(auv_xy_velo, double, 0.4) \
  /* AUV velocity in the xy plane  */ \
  CXT_MACRO_MEMBER(auv_z_accel, double, 0.15) \
  /* AUV vertical acceleration  */ \
  CXT_MACRO_MEMBER(auv_z_velo, double, 0.3) \
  /* AUV vertical velocity  */ \
  CXT_MACRO_MEMBER(auv_yaw_accel, double, 0.2) \
  /* AUV rotation acceleration  */ \
  CXT_MACRO_MEMBER(auv_yaw_velo, double, 0.4) \
  /* AUV rotation velocity  */ \
 \
  CXT_MACRO_MEMBER(mtm_fwd_accel, double, 0.2) \
  /* Move to marker forward acceleration  */ \
  CXT_MACRO_MEMBER(mtm_fwd_velo, double, 0.4) \
  /* Move to marker forward velocity  */ \
  CXT_MACRO_MEMBER(mtm_yaw_accel, double, 0.1) \
  /* Move to marker yaw acceleration  */ \
  CXT_MACRO_MEMBER(mtm_yaw_velo, double, 0.2) \
  /* Move to marker yaw velocity  */ \
 \
  CXT_MACRO_MEMBER(good_pose_dist, double, 1.8) \
  /* Good pose if marker < 1.8m away  */ \
  CXT_MACRO_MEMBER(good_obs_dist, double, 10) \
  /* Good observation if marker < 10m away  */ \
 \
  /* Global planner: */ \
  CXT_MACRO_MEMBER(global_plan_allow_mtm, bool, false) \
  /* Allow mtm recovery  */ \
  CXT_MACRO_MEMBER(global_plan_max_xy_err, double, 0.6) \
  /* Replan if xy distance > max  */ \
  CXT_MACRO_MEMBER(global_plan_max_obs_yaw_err, double, 0.2) \
  /* Start recovery if observation yaw error is > max  */ \
  CXT_MACRO_MEMBER(global_plan_target_z, double, -0.5) \
  /* Target z position  */ \
 \
  /* Local (pose) planner: */ \
  CXT_MACRO_MEMBER(pose_plan_waypoints, bool, false) \
  /* Use A* to find waypoints between targets  */ \
  CXT_MACRO_MEMBER(pose_plan_epsilon_xyz, double, 0.05) \
  /* If xyz distance < epsilon, skip this segment  */ \
  CXT_MACRO_MEMBER(pose_plan_epsilon_yaw, double, 0.05) \
  /* If yaw distance < epsilon, skip this segment  */ \
  CXT_MACRO_MEMBER(pose_plan_max_short_plan_xy, double, 2) \
  /* Build a long plan if xy distance > max  */ \
  CXT_MACRO_MEMBER(pose_plan_max_dead_reckon_dist, double, 9) \
  /* Max allowable dead reckoning distance  */ \
  CXT_MACRO_MEMBER(pose_plan_target_dist, double, 1) \
  /* Target distance in front of marker  */ \
  CXT_MACRO_MEMBER(pose_plan_pause_duration, double, 1) \
  /* Length of pause segments in local plan  */ \
 \
  /* Move to marker (mtm) planner: */ \
  CXT_MACRO_MEMBER(mtm_plan_target_dist, double, 1) \
  /* Target distance from marker */ \
 \
  /* Controller: */ \
  CXT_MACRO_MEMBER(control_use_est_yaw, bool, false) \
  /* Use estimated yaw (vs planned yaw) when computing efforts */ \
/* End of list */

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)

struct AUVContext : BaseContext
{
  AUV_NODE_PARAMS
};

#define AUV_NODE_ALL_PARAMS \
  BASE_ALL_PARAMS \
  AUV_NODE_PARAMS \
/* End of list */

}  // namespace orca_base

#endif  // ORCA_BASE__AUV_CONTEXT_HPP_
