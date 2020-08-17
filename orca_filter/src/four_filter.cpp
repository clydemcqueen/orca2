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

#include "orca_filter/filter_base.hpp"

#include "eigen3/Eigen/Dense"
#include "geometry_msgs/msg/twist.hpp"
#include "orca_shared/model.hpp"
#include "orca_shared/util.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace orca_filter
{

//==================================================================
// FourFilter -- 4dof filter with 12 dimensions
//==================================================================

constexpr int FOUR_STATE_DIM = 12;      // [x, y, ..., vx, vy, ..., ax, ay, ...]T

// Four state macros
#define fx_x x(0)
#define fx_y x(1)
#define fx_z x(2)
#define fx_yaw x(3)
#define fx_vx x(4)
#define fx_vy x(5)
#define fx_vz x(6)
#define fx_vyaw x(7)
#define fx_ax x(8)
#define fx_ay x(9)
#define fx_az x(10)
#define fx_ayaw x(11)

// Init x from pose
Eigen::VectorXd pose_to_fx(const geometry_msgs::msg::Pose & pose)
{
  Eigen::VectorXd x = Eigen::VectorXd::Zero(FOUR_STATE_DIM);

  fx_x = pose.position.x;
  fx_y = pose.position.y;
  fx_z = pose.position.z;
  fx_yaw = mw::Quaternion{pose.orientation}.yaw();

  return x;
}

// Extract pose from PoseFilter state
void pose_from_fx(const Eigen::VectorXd & x, geometry_msgs::msg::Pose & out)
{
  out.position.x = fx_x;
  out.position.y = fx_y;
  out.position.z = fx_z;

  tf2::Matrix3x3 m;
  m.setRPY(0, 0, fx_yaw);

  tf2::Quaternion q;
  m.getRotation(q);

  out.orientation = tf2::toMsg(q);
}

// Extract twist from PoseFilter state
void twist_from_fx(const Eigen::VectorXd & x, geometry_msgs::msg::Twist & out)
{
  out.linear.x = fx_vx;
  out.linear.y = fx_vy;
  out.linear.z = fx_vz;

  out.angular.x = 0;
  out.angular.y = 0;
  out.angular.z = fx_vyaw;
}

// Extract pose or twist covariance from PoseFilter covariance
void flatten_4x4_covar(const Eigen::MatrixXd & P, std::array<double, 36> & pose_covar, bool pose)
{
  // Start with identity
  Eigen::MatrixXd m = Eigen::MatrixXd::Identity(6, 6);

  // Copy values from the 4x4 into the 6x6
  int offset = pose ? 0 : 4;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      m(i < 3 ? i : i + 2, j < 3 ? j : j + 2) = P(i + offset, j + offset);
    }
  }

  // Flatten the 6x6 matrix
  flatten_6x6_covar(m, pose_covar, 0);
}

Eigen::VectorXd
four_state_residual(const Eigen::Ref<const Eigen::VectorXd> & x, const Eigen::VectorXd & mean)
{
  // Residual for all fields
  Eigen::VectorXd residual = x - mean;

  // Normalize yaw
  residual(3) = orca::norm_angle(residual(3));

  return residual;
}

Eigen::VectorXd four_state_mean(const Eigen::MatrixXd & sigma_points, const Eigen::RowVectorXd & Wm)
{
  Eigen::VectorXd mean = Eigen::VectorXd::Zero(sigma_points.rows());

  // Standard mean for all fields
  for (int64_t i = 0; i < sigma_points.cols(); ++i) {
    mean += Wm(i) * sigma_points.col(i);
  }

  // Sum the sines and cosines
  double sum_y_sin = 0.0, sum_y_cos = 0.0;

  for (int64_t i = 0; i < sigma_points.cols(); ++i) {
    sum_y_sin += Wm(i) * sin(sigma_points(3, i));
    sum_y_cos += Wm(i) * cos(sigma_points(3, i));
  }

  // Mean is arctan2 of the sums
  mean(3) = atan2(sum_y_sin, sum_y_cos);

  return mean;
}

FourFilter::FourFilter(
  const rclcpp::Logger & logger,
  const FilterContext & cxt,
  rclcpp::Publisher<orca_msgs::msg::FiducialPoseStamped>::SharedPtr filtered_odom_pub,
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub)
: FilterBase{Type::four, logger, cxt, filtered_odom_pub, tf_pub, FOUR_STATE_DIM}
{
  filter_.set_Q(Eigen::MatrixXd::Identity(FOUR_STATE_DIM, FOUR_STATE_DIM) * 0.01);

  // State transition function
  filter_.set_f_fn(
    [&cxt](const double dt, const Eigen::VectorXd & u, Eigen::Ref<Eigen::VectorXd> x)
    {
      if (cxt.predict_accel_) {
        // Assume 0 acceleration
        fx_ax = 0;
        fx_ay = 0;
        fx_az = 0;
        fx_ayaw = 0;

        if (cxt.predict_accel_control_) {
          // Add acceleration due to control
          fx_ax += u(0, 0);
          fx_ay += u(1, 0);
          fx_az += u(2, 0);
          fx_ayaw += u(3, 0);
        }

        if (cxt.predict_accel_drag_) {
          // Add acceleration due to drag
          // TODO(clyde): create & use AddLinkForce(drag_force, c_of_mass)
          //  and AddRelativeTorque(drag_torque)
          // Simple approximation:
          fx_ax += cxt.model_.drag_accel_f(fx_vx);    // TODO(clyde): f or x?
          fx_ay += cxt.model_.drag_accel_s(fx_vy);    // TODO(clyde): s or y?
          fx_az += cxt.model_.drag_accel_z(fx_vz);
          fx_ayaw += cxt.model_.drag_accel_yaw(fx_vyaw);
        }

        if (cxt.predict_accel_buoyancy_) {
          // Add acceleration due to gravity and buoyancy
          // TODO(clyde): create & use AddLinkForce(buoyancy_force, c_of_volume)
          // Simple approximation:
          fx_az -= cxt.model_.hover_accel_z();
        }
      }

      // Clamp acceleration
      fx_ax = orca::clamp(fx_ax, MAX_PREDICTED_ACCEL_XYZ);
      fx_ay = orca::clamp(fx_ay, MAX_PREDICTED_ACCEL_XYZ);
      fx_az = orca::clamp(fx_az, MAX_PREDICTED_ACCEL_XYZ);
      fx_ayaw = orca::clamp(fx_ayaw, MAX_PREDICTED_ACCEL_RPY);

      // Velocity, vx += ax * dt
      fx_vx += fx_ax * dt;
      fx_vy += fx_ay * dt;
      fx_vz += fx_az * dt;
      fx_vyaw += fx_ayaw * dt;

      // Clamp velocity
      fx_vx = orca::clamp(fx_vx, MAX_PREDICTED_VELO_XYZ);
      fx_vy = orca::clamp(fx_vy, MAX_PREDICTED_VELO_XYZ);
      fx_vz = orca::clamp(fx_vz, MAX_PREDICTED_VELO_XYZ);
      fx_vyaw = orca::clamp(fx_vyaw, MAX_PREDICTED_VELO_RPY);

      // Position, x += vx * dt
      fx_x += fx_vx * dt;
      fx_y += fx_vy * dt;
      fx_z += fx_vz * dt;
      fx_yaw = orca::norm_angle(fx_yaw + fx_vyaw * dt);
    });

  // Custom residual and mean functions
  filter_.set_r_x_fn(four_state_residual);
  filter_.set_mean_x_fn(four_state_mean);
}

void FourFilter::reset(const geometry_msgs::msg::Pose & pose)
{
  FilterBase::reset(pose_to_fx(pose));
}

void FourFilter::odom_from_filter(orca_msgs::msg::FiducialPose & filtered_odom)
{
  pose_from_fx(filter_.x(), filtered_odom.pose.pose);
  // twist_from_fx(filter_.x(), filtered_odom.twist.twist);
  flatten_4x4_covar(filter_.P(), filtered_odom.pose.covariance, true);
  // flatten_4x4_covar(filter_.P(), filtered_odom.twist.covariance, false);
}

Measurement FourFilter::to_measurement(
  const orca_msgs::msg::Depth & depth,
  const mw::Observations & observations) const
{
  Measurement m;
  m.init_z(depth, observations,
    [](const Eigen::Ref<const Eigen::VectorXd> & x, Eigen::Ref<Eigen::VectorXd> z)
    {
      z(0) = fx_z;
    });
  return m;
}

Measurement FourFilter::to_measurement(
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose,
  const mw::Observations & observations) const
{
  Measurement m;
  m.init_4dof(pose, observations,
    [](const Eigen::Ref<const Eigen::VectorXd> & x, Eigen::Ref<Eigen::VectorXd> z)
    {
      z(0) = fx_x;
      z(1) = fx_y;
      z(2) = fx_z;
      z(3) = fx_yaw;
    });
  return m;
}

}  // namespace orca_filter
