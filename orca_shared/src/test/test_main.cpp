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
#include "orca_shared/test.hpp"

sensor_msgs::msg::CameraInfo make_camera_info()
{
  sensor_msgs::msg::CameraInfo camera_info;
  camera_info.width = 800;
  camera_info.height = 600;
  camera_info.distortion_model = "plumb_bob";

  camera_info.k[0] = camera_info.p[0] = 474.89673285067175;
  camera_info.k[2] = camera_info.p[2] = 400.5;
  camera_info.k[4] = camera_info.p[4] = 474.89673285067175;
  camera_info.k[5] = camera_info.p[5] = 300.5;
  camera_info.k[8] = camera_info.p[8] = 1;

  camera_info.r[0] = 1;
  camera_info.r[4] = 1;
  camera_info.r[8] = 1;

  return camera_info;
}

geometry_msgs::msg::Pose make_cam_f_base()
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 1;
  pose.orientation.x = 0.5;
  pose.orientation.y = -0.5;
  pose.orientation.z = 0.5;
  pose.orientation.w = -0.5;
  return pose;
}

int main(int argc, char ** argv)
{
  if (test_mw_move() &&
    test_mw_header() &&
    test_mw_observer() &&
    test_mw_pose_segment() &&
    test_mw_pose_body() &&
    test_mw_roundtrip())
  {
    std::cout << "all successful" << std::endl;
  }
}
