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

#ifndef ORCA_BASE__POSE_SEGMENT_HPP_
#define ORCA_BASE__POSE_SEGMENT_HPP_

#include <memory>
#include <string>

#include "orca_base/segment_common.hpp"
#include "orca_shared/mw/mw.hpp"

namespace orca_base
{

//=====================================================================================
// plan_pose_sync
//=====================================================================================

/**
 * Trapezoidal velocity motion planner for 4DoF poses
 * All DoF are moving through the phases at the same time
 *
 * @param cxt AUV context object with various parameters
 * @param p0 In: start of motion
 * @param p1 Out: pose and time at end of phase 1
 * @param p2 Out: pose and time at end of phase 2
 * @param p3 In: pose at end of phase 3, out: time at end of phase 3
 * @param a0 Out: acceleration, apply from p0.t to p1.t, and decelerate from p2.t to p3.t
 * @param v1 Out: peak velocity runs from p1.t to p2.t
 */
void plan_pose_sync(
  const AUVContext & cxt, const mw::PoseStamped & p0,
  mw::PoseStamped & p1, mw::PoseStamped & p2, mw::PoseStamped & p3,
  mw::Acceleration & a0, mw::Twist & v1);

//=====================================================================================
// Pose segments plan motion based on poses
//=====================================================================================

class PoseSegmentBase : public SegmentBase
{
protected:
  mw::PoseStamped plan_;      // Planned pose, incremented with each call to advance()
  mw::Pose goal_;             // Goal pose

  mw::Twist twist_;           // Velocity in the world frame
  mw::Acceleration ff_;       // Acceleration in the world frame

  uint8_t phase_;             // Motion phase: accel, constant_v, decel

public:
  PoseSegmentBase(const AUVContext & cxt, uint8_t type, mw::PoseStamped start, mw::Pose goal);

  const mw::PoseStamped & plan() const {return plan_;}

  const mw::Pose & goal() const {return goal_;}

  const mw::Twist & twist() const {return twist_;}

  const mw::Acceleration & ff() const {return ff_;}

  uint8_t phase() const {return phase_;}
};

//=====================================================================================
// Pause segments stay in one pose for a period of time
//=====================================================================================

class Pause : public PoseSegmentBase
{
  rclcpp::Duration pause_duration_;     // Time remaining

public:
  Pause(
    const AUVContext & cxt, const mw::PoseStamped & start,
    const rclcpp::Duration & pause_duration);

  std::string to_str() override;

  bool advance(const rclcpp::Duration & d) override;
};

//=====================================================================================
// Trap2: implement a trapezoidal velocity motion segment
//=====================================================================================

class Trap2 : public PoseSegmentBase
{
  mw::PoseStamped p0_;    // Start pose
  mw::PoseStamped p1_;
  mw::PoseStamped p2_;
  mw::PoseStamped p3_;    // End pose
  mw::Acceleration a0_;   // Acceleration at p0_.t
  mw::Twist v1_;          // Velocity at p1_.t

public:
  Trap2(const AUVContext & cxt, uint8_t type, const mw::PoseStamped & start, const mw::Pose & goal);

  /**
   * Duration of motion
   * @return duration of motion
   */
  rclcpp::Duration duration() const;

  std::string to_str() override;

  /**
   * Advance plan by d TODO(clyde): pass in t, not d
   * @param d time increment
   * @return true if plan was advanced, false if plan is complete
   */
  bool advance(const rclcpp::Duration & d) override;

  /**
   * Construct a segment that moves up or down
   * @param cxt AUV context
   * @param plan In/out: plan
   * @param z In: vertical distance to travel
   * @return Trap2 segment
   */
  static std::shared_ptr<Trap2>
  make_vertical(const AUVContext & cxt, mw::PoseStamped & plan, double z);

  /**
   * Construct a segment that rotates
   * @param cxt AUV context
   * @param plan In/out: plan
   * @param yaw In: incremental angle
   * @return Trap2 segment
   */
  static std::shared_ptr<Trap2>
  make_rotate(const AUVContext & cxt, mw::PoseStamped & plan, double yaw);

  /**
   * Construct a segment that moves in a line
   * @param cxt AUV context
   * @param plan In/out: plan
   * @param x In: x distance
   * @param y In: y distance
   * @return Trap2 segment
   */
  static std::shared_ptr<Trap2>
  make_line(const AUVContext & cxt, mw::PoseStamped & plan, double x, double y);

  /**
   * Construct a segment that rotates and moves in x, y and z
   * @param cxt AUV context
   * @param plan In/out: plan
   * @param goal In: goal pose
   * @return Trap2 segment
   */
  static std::shared_ptr<Trap2>
  make_pose(const AUVContext & cxt, mw::PoseStamped & plan, const mw::Pose & goal);
};

}  // namespace orca_base

#endif  // ORCA_BASE__POSE_SEGMENT_HPP_
