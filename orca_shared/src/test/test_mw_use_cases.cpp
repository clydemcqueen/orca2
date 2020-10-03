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

#include "orca_shared/mw/mw.hpp"
#include "orca_shared/test.hpp"

template<typename T>
inline bool close_enough(const T & x, const T & y)
{
  return abs(x - y) < 0.001;
}

bool test_mw_move()
{
  std::cout << "=== TEST MOVE ===" << std::endl;

  mw::Point p0{};
  rclcpp::Duration d{1, 0};
  mw::Twist v0{1, 0, 0, 0};
  mw::Acceleration a{0, 1, 0, 0};

  auto p1 = p0.motion(d, v0, a);

  std::cout << "p0 " << p0 << ", p1 " << p1 << std::endl;

  if (!close_enough(p1.x(), 1.) ||
    !close_enough(p1.y(), 0.5) ||
    !close_enough(p1.z(), 0.))
  {
    std::cout << "failure" << std::endl;
    return false;
  }

  std::cout << "success" << std::endl;
  return true;
}

bool test_mw_header()
{
  std::cout << "=== TEST HEADER ===" << std::endl;

  mw::Header h1{};
  h1.frame_id() = "map";
  h1.t() = rclcpp::Time{0, 999, RCL_ROS_TIME};
  std::cout << h1 << std::endl;

  if (h1.frame_id() != "map" || h1.t().nanoseconds() != 999) {
    std::cout << "failure" << std::endl;
    return false;
  }

  std::cout << "success" << std::endl;
  return true;
}

bool test_mw_observer()
{
  std::cout << "=== TEST OBSERVER ===" << std::endl;

  image_geometry::PinholeCameraModel cam_model;
  cam_model.fromCameraInfo(make_camera_info());

  tf2::Transform t_base_cam;
  tf2::fromMsg(make_cam_f_base(), t_base_cam);

  const mw::Observer observer{0.2, cam_model, t_base_cam};
  std::cout << observer << std::endl;

  const mw::PolarObservation po1{1, 10, 0};
  const mw::PolarObservation po2{2, 10, 0.5};

  mw::Observations observations{observer};
  observations.add_polar(po1);
  observations.add_polar(po2);
  std::cout << observations << std::endl;

  std::cout << "success" << std::endl;
  return true;
}

bool test_mw_pose_segment()
{
  std::cout << "=== TEST POSE SEGMENT ===" << std::endl;

  mw::Header header{rclcpp::Time{0, 999, RCL_ROS_TIME}, "map"};
  mw::Pose pose{0, 0, 0, 0};
  mw::PoseStamped plan{header, pose};

  mw::Pose goal{plan.pose()};
  goal.position().x() = 5;
  goal.position().y() = 10;
  goal.orientation().yaw(0.5);
  std::cout << goal << std::endl;

  const int num_iter = 10;
  auto xy_distance = goal.position().distance_xy(plan.pose().position());
  auto yaw_distance = goal.orientation().distance_yaw(plan.pose().orientation());
  auto angle_to_goal = std::atan2(goal.position().y() - plan.pose().position().y(),
      goal.position().x() - plan.pose().position().x());

  for (int i = 0; i < num_iter; ++i) {
    plan.pose().position().x() += std::cos(angle_to_goal) * xy_distance / num_iter;
    plan.pose().position().y() += std::sin(angle_to_goal) * xy_distance / num_iter;
    plan.pose().orientation().yaw(plan.pose().orientation().yaw() + yaw_distance / num_iter);
    std::cout << plan.pose() << std::endl;
  }

  std::cout << "success" << std::endl;
  return true;
}

bool test_mw_pose_body()
{
  std::cout << "=== TEST POSE SEGMENT ===" << std::endl;

  mw::Pose start{mw::Point{0, 0, 0}, mw::Quaternion{0, 0, 0.1}};
  mw::PoseBody delta{1, 0, 0, 0};
  mw::Pose result = start + delta;
  std::cout << result << std::endl;

  mw::Pose expected{mw::Point{0.995, 0.1, 0}, mw::Quaternion{0, 0, 0.1}};
  if (!close_enough(result.x(), expected.x()) ||
    !close_enough(result.y(), expected.y()) ||
    !close_enough(result.z(), expected.z()) ||
    !close_enough(result.roll(), expected.roll()) ||
    !close_enough(result.pitch(), expected.pitch()) ||
    !close_enough(result.yaw(), expected.yaw()))
  {
    std::cout << "failure" << std::endl;
    return false;
  }

  std::cout << "success" << std::endl;
  return true;
}
