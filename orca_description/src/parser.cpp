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

#include "orca_description/parser.hpp"

#include <string>

#include "urdf/model.h"

namespace orca_description
{

bool Parser::parse()
{
  urdf::Model model;

  auto result = model.initFile(filename) &&
    get_joint(model, barometer_joint, t_baro_base) &&
    get_joint(model, forward_camera_joint, t_fcam_base) &&
    get_joint(model, left_camera_joint, t_lcam_base) &&
    get_joint(model, right_camera_joint, t_rcam_base);

  t_base_baro = t_baro_base.inverse();
  t_base_fcam = t_fcam_base.inverse();
  t_base_lcam = t_lcam_base.inverse();
  t_base_rcam = t_rcam_base.inverse();

  return result;
}

bool
Parser::get_joint(const urdf::Model & model, const std::string & joint_name, tf2::Transform & t)
{
  auto joint = model.getJoint(joint_name);

  if (!joint) {
    std::cout << "joint " << joint_name << " missing" << std::endl;
    return false;
  }

  if (joint->parent_link_name != base_link) {
    std::cout << "joint " << joint_name << " expected parent " << base_link <<
      ", found " << joint->parent_link_name << std::endl;
    return false;
  }

  auto pose = joint->parent_to_joint_origin_transform;

  tf2::Transform t2 = tf2::Transform{
    tf2::Quaternion{pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w},
    tf2::Vector3{pose.position.x, pose.position.y, pose.position.z}};

  t = t2.inverse();

  return true;
}

}  // namespace orca_description
