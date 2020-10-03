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

#ifndef ORCA_DESCRIPTION__PARSER_HPP_
#define ORCA_DESCRIPTION__PARSER_HPP_

#include <string>

#include "tf2/LinearMath/Transform.h"
#include "urdf/model.h"

namespace orca_description
{
constexpr const char * filename = "install/orca_description/share/orca_description/urdf/orca.urdf";
constexpr const char * base_link = "base_link";
constexpr const char * barometer_joint = "baro_joint";

// Note that these are x_camera_FRAME_joint, not x_camera_joint
// These relate the camera image to the base link
constexpr const char * forward_camera_joint = "forward_camera_frame_joint";
constexpr const char * left_camera_joint = "left_camera_frame_joint";
constexpr const char * right_camera_joint = "right_camera_frame_joint";

struct Parser
{
  // Transform base_f_sensor_frame for all sensors
  tf2::Transform t_baro_base{};
  tf2::Transform t_fcam_base{};
  tf2::Transform t_lcam_base{};
  tf2::Transform t_rcam_base{};

  // Inverse transforms
  tf2::Transform t_base_baro{};
  tf2::Transform t_base_fcam{};
  tf2::Transform t_base_lcam{};
  tf2::Transform t_base_rcam{};

  // Parse URDF and set t_foo_base, return true if OK
  bool parse();

  // Get a particular joint, return true if OK
  static bool
  get_joint(const urdf::Model & model, const std::string & joint_name, tf2::Transform & t);
};

}  // namespace orca_description

#endif  // ORCA_DESCRIPTION__PARSER_HPP_
