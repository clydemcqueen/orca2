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

#ifndef ORCA_SHARED__UTIL_HPP_
#define ORCA_SHARED__UTIL_HPP_

#include <cstdint>
#include <cmath>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "tf2/LinearMath/Transform.h"

namespace orca
{

template<typename T>
constexpr T clamp(const T v, const T min, const T max)
{
  return v > max ? max : (v < min ? min : v);
}

template<typename T>
constexpr T clamp(const T v, const T minmax)
{
  return clamp(v, -minmax, minmax);
}

template<typename T>
constexpr T dead_band(const T v, const T d)
{
  return v < d && v > -d ? 0 : v;
}

template<typename A, typename B>
constexpr B scale(const A a, const A a_min, const A a_max, const B b_min, const B b_max)
{
  return clamp(
    static_cast<B>(b_min + static_cast<double>(b_max - b_min) / (a_max - a_min) * (a - a_min)),
    b_min,
    b_max);
}

#if 1

// Move an angle to the region [-M_PI, M_PI)
constexpr double norm_angle(double a)
{
  if (a < -M_PI || a > M_PI) {
    // Force to [-2PI, 2PI)
    a = fmod(a, 2 * M_PI);

    // Move to [-PI, PI)
    if (a < -M_PI) {
      a += 2 * M_PI;
    } else if (a > M_PI) {
      a -= 2 * M_PI;
    }
  }

  return a;
}

#else
// Move an angle to the region [-M_PI, M_PI]
constexpr double norm_angle(double a)
{
  while (a < -M_PI) {
    a += 2 * M_PI;
  }
  while (a > M_PI) {
    a -= 2 * M_PI;
  }

  return a;
}
#endif

// Compute a 2d point in a rotated frame (v' = R_transpose * v)
void rotate_frame(double x, double y, double theta, double & x_r, double & y_r);

// Get roll, pitch and yaw from a quaternion
void get_rpy(const geometry_msgs::msg::Quaternion & q, double & roll, double & pitch, double & yaw);

// Get yaw from a quaternion
double get_yaw(const geometry_msgs::msg::Quaternion & q);

// Various to_str functions
std::string to_str_rpy(const tf2::Transform & t);

std::string to_str_q(const tf2::Transform & t);

std::string to_str(const rclcpp::Time & t);

std::string to_str(const builtin_interfaces::msg::Time & t);

// True if a rclcpp::Time is valid (non-zero)
bool valid_stamp(const rclcpp::Time & stamp);

tf2::Transform pose_to_transform(const geometry_msgs::msg::Pose & pose);

geometry_msgs::msg::Pose transform_to_pose(const tf2::Transform & transform);

}  // namespace orca

#endif  // ORCA_SHARED__UTIL_HPP_
