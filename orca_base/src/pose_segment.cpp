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

#include "orca_base/pose_segment.hpp"

#include <memory>
#include <iomanip>
#include <iostream>
#include <utility>
#include <string>

#include "orca_msgs/msg/control.hpp"
#include "orca_shared/util.hpp"

namespace orca_base
{

//=====================================================================================
// plan_pose_sync
//=====================================================================================

void plan_pose_sync(
  const AUVContext & cxt, const mw::PoseStamped & p0,
  mw::PoseStamped & p1, mw::PoseStamped & p2, mw::PoseStamped & p3,
  mw::Acceleration & a0, mw::Twist & v1)
{
  /* Notes on motion in the xy plane:
   *
   * If we move in x and y separately we end up with higher 2D velocity for diagonal motion
   * (e.g., x+5, y+5) than for motion along an axis (e.g., x+0, y+5). Avoid this by computing
   * xy motion together.
   *
   * We do still have cases where we end up with higher 3D velocity for diagonal motion
   * (e.g., x+5, y+5, z+5). This is OK because xy and z motion use different thrusters.
   *
   * We are left with the case where xy motion and yaw motion, which use the same thrusters,
   * may combine to saturate the thrusters. For now we avoid this by picking the right values
   * for auv_xy_accel, auv_xy_velo, auv_yaw_accel, auv_yaw_velo, and xy_limit.
   */

  // Create the fastest plans
  double dxy = p3.pose().position().distance_xy(p0.pose().position());
  FastPlan xy_fast(false, dxy, cxt.auv_xy_accel_, cxt.auv_xy_velo_);
  FastPlan z_fast(false, p3.pose().z() - p0.pose().z(), cxt.auv_z_accel_, cxt.auv_z_velo_);
  FastPlan yaw_fast(true, p3.pose().yaw() - p0.pose().yaw(), cxt.auv_yaw_accel_, cxt.auv_yaw_velo_);

  // Find the longest ramp and run times
  double t_ramp = xy_fast.t_ramp;
  if (t_ramp < z_fast.t_ramp) {t_ramp = z_fast.t_ramp;}
  if (t_ramp < yaw_fast.t_ramp) {t_ramp = yaw_fast.t_ramp;}

  double t_run = xy_fast.t_run;
  if (t_run < z_fast.t_run) {t_run = z_fast.t_run;}
  if (t_run < yaw_fast.t_run) {t_run = yaw_fast.t_run;}

  // Create plans where the phases are synchronized
  SyncPlan xy_sync(false, dxy, t_ramp, t_run);
  SyncPlan z_sync(false, p3.pose().z() - p0.pose().z(), t_ramp, t_run);
  SyncPlan yaw_sync(true, p3.pose().yaw() - p0.pose().yaw(), t_ramp, t_run);

  // Save results
  p1.header().t() = p0.header().t() + rclcpp::Duration::from_seconds(t_ramp);
  p2.header().t() = p1.header().t() + rclcpp::Duration::from_seconds(t_run);
  p3.header().t() = p2.header().t() + rclcpp::Duration::from_seconds(t_ramp);

  auto angle_to_goal = atan2(p3.pose().y() - p0.pose().y(), p3.pose().x() - p0.pose().x());
  auto xf = cos(angle_to_goal);   // x fraction of xy motion
  auto yf = sin(angle_to_goal);   // y fraction of xy motion

  p1.pose() =
    p0.pose() + mw::Pose(xf * xy_sync.d_ramp, yf * xy_sync.d_ramp, z_sync.d_ramp, yaw_sync.d_ramp);
  p2.pose() =
    p1.pose() + mw::Pose(xf * xy_sync.d_run, yf * xy_sync.d_run, z_sync.d_run, yaw_sync.d_run);

  a0.x() = xf * xy_sync.a;
  a0.y() = yf * xy_sync.a;
  a0.z() = z_sync.a;
  a0.yaw() = yaw_sync.a;

  v1.x() = xf * xy_sync.v;
  v1.y() = yf * xy_sync.v;
  v1.z() = z_sync.v;
  v1.yaw() = yaw_sync.v;
}

//=====================================================================================
// PoseSegmentBase
//=====================================================================================

PoseSegmentBase::PoseSegmentBase(
  const AUVContext & cxt, uint8_t type, mw::PoseStamped start,
  mw::Pose goal)
: SegmentBase{cxt, type},
  plan_{std::move(start)},
  goal_{goal}
{
  // Default ff includes acceleration to counteract buoyancy
  ff_ = mw::Acceleration{0, 0, cxt_.hover_accel_z(), 0};
}

//=====================================================================================
// Pause
//=====================================================================================

Pause::Pause(
  const AUVContext & cxt, const mw::PoseStamped & start,
  const rclcpp::Duration & pause_duration)
: PoseSegmentBase{cxt, orca_msgs::msg::MissionState::PAUSE, start, start.pose()},
  pause_duration_{pause_duration} {}

std::string Pause::to_str()
{
  std::stringstream ss;
  ss << "pause for " << pause_duration_.seconds() << " seconds";
  return ss.str();
}

bool Pause::advance(const rclcpp::Duration & d)
{
  // Update plan
  plan_.header().t() = plan_.header().t() + d;

  // Count down time remaining
  pause_duration_ = pause_duration_ - d;

  return pause_duration_.nanoseconds() > 0;
}

//=====================================================================================
// Trap2
//=====================================================================================

Trap2::Trap2(
  const AUVContext & cxt, uint8_t type, const mw::PoseStamped & start,
  const mw::Pose & goal)
: PoseSegmentBase{cxt, type, start, goal}
{
  p0_ = start;
  // plan_pose_sync will set p3_.header().t()
  p3_.pose() = goal;

  // Create a synchronized motion plan
  plan_pose_sync(cxt_, p0_, p1_, p2_, p3_, a0_, v1_);
}

rclcpp::Duration Trap2::duration() const
{
  return p3_.header().t() - p0_.header().t();
}

std::string Trap2::to_str()
{
  std::stringstream ss;
  ss << std::fixed << std::setprecision(3) <<
    type_name() << ", accel dt: " << (p1_.header().t() - p0_.header().t()).seconds() <<
    ", run dt: " << (p2_.header().t() - p1_.header().t()).seconds() <<
    ", decel dt: " << (p3_.header().t() - p2_.header().t()).seconds();
  return ss.str();
}

bool Trap2::advance(const rclcpp::Duration & d)
{
  plan_.header().t() = plan_.header().t() + d;

  // Update total acceleration, twist_ and pose
  mw::Acceleration accel;
  if (plan_.header().t() >= p0_.header().t() && plan_.header().t() < p1_.header().t()) {
    // Phase 1: accelerate
    phase_ = orca_msgs::msg::MissionState::PHASE_TRAP_ACCEL;
    accel = a0_;
    twist_ = mw::Twist{}.motion(plan_.header().t() - p0_.header().t(), accel);
    plan_.pose() = p0_.pose().motion(plan_.header().t() - p0_.header().t(), {}, accel);
  } else if (plan_.header().t() < p2_.header().t()) {
    // Phase 2: run at constant velocity
    phase_ = orca_msgs::msg::MissionState::PHASE_TRAP_CONSTANT_V;
    accel = {};
    twist_ = v1_;
    plan_.pose() = p1_.pose().motion(plan_.header().t() - p1_.header().t(), v1_, accel);
  } else if (plan_.header().t() < p3_.header().t()) {
    // Phase 3: decelerate
    phase_ = orca_msgs::msg::MissionState::PHASE_TRAP_DECEL;
    accel = -a0_;
    twist_ = v1_.motion(plan_.header().t() - p2_.header().t(), accel);
    plan_.pose() = p2_.pose().motion(plan_.header().t() - p2_.header().t(), v1_, accel);
  } else {
    // Outside the plan
    return false;
  }

  // Compute acceleration due to drag
  // Drag must be computed in the body frame
  double twist_forward, twist_strafe;
  orca::rotate_frame(twist_.x(), twist_.y(), plan_.pose().yaw(), twist_forward, twist_strafe);
  auto drag_forward = cxt_.drag_accel_f(twist_forward);
  auto drag_strafe = cxt_.drag_accel_s(twist_strafe);
  mw::Acceleration drag;
  orca::rotate_frame(drag_forward, drag_strafe, -plan_.pose().yaw(), drag.x(), drag.y());

  // Continue with z and yaw drag
  drag.z() = cxt_.drag_accel(twist_.z(), cxt_.drag_const_z());
  drag.yaw() = cxt_.drag_accel_yaw(twist_.yaw());

  // Feedforward (acceleration due to thrust) is total acceleration minus drag
  ff_ = accel - drag;

  // Add hover to feedforward
  ff_.z() += cxt_.hover_accel_z();

  return true;
}

std::shared_ptr<Trap2>
Trap2::make_vertical(const AUVContext & cxt, mw::PoseStamped & plan, double z)
{
  auto goal = plan.pose();
  goal.z() = z;
  auto
    result = std::make_shared<Trap2>(cxt, orca_msgs::msg::MissionState::POSE_VERTICAL, plan, goal);
  plan.pose() = goal;
  plan.header().t() = plan.header().t() + result->duration();
  return result;
}

std::shared_ptr<Trap2>
Trap2::make_rotate(const AUVContext & cxt, mw::PoseStamped & plan, double yaw)
{
  auto goal = plan.pose();
  goal.yaw(yaw);
  auto result = std::make_shared<Trap2>(cxt, orca_msgs::msg::MissionState::POSE_ROTATE, plan, goal);
  plan.pose() = goal;
  plan.header().t() = plan.header().t() + result->duration();
  return result;
}

std::shared_ptr<Trap2>
Trap2::make_line(const AUVContext & cxt, mw::PoseStamped & plan, double x, double y)
{
  auto goal = plan.pose();
  goal.x() = x;
  goal.y() = y;
  auto result = std::make_shared<Trap2>(cxt, orca_msgs::msg::MissionState::POSE_LINE, plan, goal);
  plan.pose() = goal;
  plan.header().t() = plan.header().t() + result->duration();
  return result;
}

std::shared_ptr<Trap2>
Trap2::make_pose(const AUVContext & cxt, mw::PoseStamped & plan, const mw::Pose & goal)
{
  auto result = std::make_shared<Trap2>(cxt, orca_msgs::msg::MissionState::POSE_COMBO, plan, goal);
  plan.pose() = goal;
  plan.header().t() = plan.header().t() + result->duration();
  return result;
}

}  // namespace orca_base
