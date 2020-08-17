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

#include <string>
#include <utility>

#include "orca_filter/filter_base.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace orca_filter
{

//==================================================================
// Measurement
//==================================================================

void Measurement::init_z(
  const orca_msgs::msg::Depth & depth,
  const mw::Observations & observations, ukf::MeasurementFn h_fn)
{
  type_ = Type::depth;
  stamp_ = depth.header.stamp;
  observations_ = observations;

  z_ = Eigen::VectorXd(1);
  z_ << depth.z;

  R_ = Eigen::MatrixXd(1, 1);
  R_ << depth.z_variance;

  h_fn_ = std::move(h_fn);

  // Use the standard residual and mean functions for depth measurements
  r_z_fn_ = ukf::residual;
  mean_z_fn_ = ukf::unscented_mean;
}

void Measurement::init_4dof(
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose,
  const mw::Observations & observations, ukf::MeasurementFn h_fn)
{
  type_ = Type::four;
  stamp_ = pose.header.stamp;
  observations_ = observations;

  tf2::Transform t_map_base;
  tf2::fromMsg(pose.pose.pose, t_map_base);

  tf2Scalar roll, pitch, yaw;
  t_map_base.getBasis().getRPY(roll, pitch, yaw);

  z_ = Eigen::VectorXd(4);
  z_ << t_map_base.getOrigin().x(), t_map_base.getOrigin().y(), t_map_base.getOrigin().z(), yaw;

  R_ = Eigen::MatrixXd(4, 4);
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      // Copy rows {0, 1, 2, 5} and cols {0, 1, 2, 5}
      R_(i, j) = pose.pose.covariance[(i < 3 ? i : 5) * 6 + (j < 3 ? j : 5)];
    }
  }

  h_fn_ = std::move(h_fn);

  // Use a custom residual and mean functions for pose measurements
  r_z_fn_ = four_state_residual;
  mean_z_fn_ = four_state_mean;
}

void Measurement::init_6dof(
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose,
  const mw::Observations & observations, ukf::MeasurementFn h_fn)
{
  type_ = Type::six;
  stamp_ = pose.header.stamp;
  observations_ = observations;

  tf2::Transform t_map_base;
  tf2::fromMsg(pose.pose.pose, t_map_base);

  tf2Scalar roll, pitch, yaw;
  t_map_base.getBasis().getRPY(roll, pitch, yaw);

  z_ = Eigen::VectorXd(6);
  z_ << t_map_base.getOrigin().x(), t_map_base.getOrigin().y(), t_map_base.getOrigin().z(),
    roll, pitch, yaw;

  R_ = Eigen::MatrixXd(6, 6);
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      R_(i, j) = pose.pose.covariance[i * 6 + j];
    }
  }

  h_fn_ = std::move(h_fn);

  // Use a custom residual and mean functions for pose measurements
  r_z_fn_ = six_state_residual;
  mean_z_fn_ = six_state_mean;
}

std::string Measurement::name() const
{
  if (type_ == Type::depth) {
    return "z";
  } else if (type_ == Type::four) {
    return "4dof";
  } else {
    return "6dof";
  }
}

}  // namespace orca_filter
