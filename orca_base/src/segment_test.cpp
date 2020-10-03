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

#include <iostream>

#include "orca_base/pose_segment.hpp"
#include "orca_shared/util.hpp"

#define CLOSE_ENOUGH(x) (abs(x) < 0.001)

void segment_test()
{
  std::cout << "=== TRAP2 SEGMENT TEST ===" << std::endl;

  mw::PoseStamped p0, p1, p2, p3;

  p3.pose().position().x() = -5;
  p3.pose().position().y() = 0;
  p3.pose().position().z() = 1;
  p3.pose().orientation().yaw(2);

  auto p3_save = p3;

  orca_base::AUVContext cxt{};

  mw::Acceleration a;
  mw::Twist v;

  orca_base::plan_pose_sync(cxt, p0, p1, p2, p3, a, v);

  std::cout << "p0: " << p0 << std::endl;
  std::cout << "p1: " << p1 << std::endl;
  std::cout << "p2: " << p2 << std::endl;
  std::cout << "p3: " << p3 << std::endl;
  std::cout << "a0: " << a << std::endl;
  std::cout << "v1: " << v << std::endl;

  // Make sure plan_sync didn't screw with the original pose
  assert(CLOSE_ENOUGH(p3_save.pose().position().x() - p3.pose().position().x()));
  assert(CLOSE_ENOUGH(p3_save.pose().position().y() - p3.pose().position().y()));
  assert(CLOSE_ENOUGH(p3_save.pose().position().z() - p3.pose().position().z()));
  assert(CLOSE_ENOUGH(
      orca::norm_angle(p3_save.pose().orientation().yaw() - p3.pose().orientation().yaw())));

  auto ramp = p1.pose() - p0.pose();
  auto run = p2.pose() - p1.pose();
  auto check = p0.pose() + ramp + run + ramp;

  // Make sure the sums work out
  assert(CLOSE_ENOUGH(p3_save.pose().position().x() - check.position().x()));
  assert(CLOSE_ENOUGH(p3_save.pose().position().y() - check.position().y()));
  assert(CLOSE_ENOUGH(p3_save.pose().position().z() - check.position().z()));
  assert(
    CLOSE_ENOUGH(orca::norm_angle(p3_save.pose().orientation().yaw() - check.orientation().yaw())));

  // TODO(clyde): check against known results

  std::cout << "success" << std::endl;
}
