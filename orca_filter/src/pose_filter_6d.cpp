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

#include <utility>

#include "orca_filter/pose_filter_base.hpp"

#include "eigen3/Eigen/Dense"
#include "geometry_msgs/msg/twist.hpp"
#include "orca_shared/model.hpp"
#include "orca_shared/util.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace orca_filter
{

//==================================================================
// PoseFilter6D -- 6dof filter with 18 dimensions
//==================================================================

constexpr int NUM_VAR = 6;
constexpr int NUM_DER = 3;
constexpr int STATE_DIM = NUM_VAR * NUM_DER;  // [x, y, ..., vx, vy, ..., ax, ay, ...]T

// State macros
#define px_x x(0)
#define px_y x(1)
#define px_z x(2)
#define px_roll x(3)
#define px_pitch x(4)
#define px_yaw x(5)
#define px_vx x(6)
#define px_vy x(7)
#define px_vz x(8)
#define px_vroll x(9)
#define px_vpitch x(10)
#define px_vyaw x(11)
#define px_ax x(12)
#define px_ay x(13)
#define px_az x(14)
#define px_aroll x(15)
#define px_apitch x(16)
#define px_ayaw x(17)

// Init x from pose
Eigen::VectorXd pose_to_px(const geometry_msgs::msg::Pose & pose)
{
  Eigen::VectorXd x = Eigen::VectorXd::Zero(STATE_DIM);

  px_x = pose.position.x;
  px_y = pose.position.y;
  px_z = pose.position.z;

  mw::Quaternion q{pose.orientation};
  px_roll = q.roll();
  px_pitch = q.pitch();
  px_yaw = q.yaw();

  return x;
}

// Extract pose from PoseFilter6D state
void pose_from_px(const Eigen::VectorXd & x, geometry_msgs::msg::Pose & out)
{
  out.position.x = px_x;
  out.position.y = px_y;
  out.position.z = px_z;

  tf2::Matrix3x3 m;
  m.setRPY(px_roll, px_pitch, px_yaw);

  tf2::Quaternion q;
  m.getRotation(q);

  out.orientation = tf2::toMsg(q);
}

// Extract twist from PoseFilter6D state
void twist_from_px(const Eigen::VectorXd & x, geometry_msgs::msg::Twist & out)
{
  out.linear.x = px_vx;
  out.linear.y = px_vy;
  out.linear.z = px_vz;

  out.angular.x = px_vroll;
  out.angular.y = px_vpitch;
  out.angular.z = px_vyaw;
}

// Extract pose covariance from PoseFilter6D covariance
void pose_covar_from_pP(const Eigen::MatrixXd & P, std::array<double, 36> & pose_covar)
{
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      pose_covar[i * 6 + j] = P(i, j);  // [0:5, 0:5]
    }
  }
}

// Extract twist covariance from PoseFilter6D covariance
void twist_covar_from_pP(const Eigen::MatrixXd & P, std::array<double, 36> & twist_covar)
{
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      twist_covar[i * 6 + j] = P(i + 6, j + 6);  // [6:12, 6:12]
    }
  }
}

Eigen::VectorXd
six_state_residual(const Eigen::Ref<const Eigen::VectorXd> & x, const Eigen::VectorXd & mean)
{
  // Residual for all fields
  Eigen::VectorXd residual = x - mean;

  // Normalize roll, pitch and yaw
  residual(3) = orca::norm_angle(residual(3));
  residual(4) = orca::norm_angle(residual(4));
  residual(5) = orca::norm_angle(residual(5));

  return residual;
}

Eigen::VectorXd six_state_mean(const Eigen::MatrixXd & sigma_points, const Eigen::RowVectorXd & Wm)
{
  Eigen::VectorXd mean = Eigen::VectorXd::Zero(sigma_points.rows());

  // Standard mean for all fields
  for (int64_t i = 0; i < sigma_points.cols(); ++i) {
    mean += Wm(i) * sigma_points.col(i);
  }

  // Sum the sines and cosines
  double sum_r_sin = 0.0, sum_r_cos = 0.0;
  double sum_p_sin = 0.0, sum_p_cos = 0.0;
  double sum_y_sin = 0.0, sum_y_cos = 0.0;

  for (int64_t i = 0; i < sigma_points.cols(); ++i) {
    sum_r_sin += Wm(i) * sin(sigma_points(3, i));
    sum_r_cos += Wm(i) * cos(sigma_points(3, i));

    sum_p_sin += Wm(i) * sin(sigma_points(4, i));
    sum_p_cos += Wm(i) * cos(sigma_points(4, i));

    sum_y_sin += Wm(i) * sin(sigma_points(5, i));
    sum_y_cos += Wm(i) * cos(sigma_points(5, i));
  }

  // Mean is arctan2 of the sums
  mean(3) = atan2(sum_r_sin, sum_r_cos);
  mean(4) = atan2(sum_p_sin, sum_p_cos);
  mean(5) = atan2(sum_y_sin, sum_y_cos);

  return mean;
}

PoseFilter6D::PoseFilter6D(
  const rclcpp::Logger & logger,
  const PoseFilterContext & cxt,
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr filtered_pose_pub,
  rclcpp::Publisher<orca_msgs::msg::FiducialPoseStamped>::SharedPtr filtered_fp_pub,
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub)
: PoseFilterBase{Type::pose_6d, logger, cxt, std::move(filtered_pose_pub),
    std::move(filtered_fp_pub), std::move(tf_pub), STATE_DIM}
{
}

void PoseFilter6D::init(const geometry_msgs::msg::Pose & pose)
{
  PoseFilterBase::init(pose_to_px(pose));

  // Process noise
  Eigen::VectorXd variances(NUM_VAR);
  variances << cxt_.ukf_var_x_, cxt_.ukf_var_y_, cxt_.ukf_var_z_,
    cxt_.ukf_var_roll_, cxt_.ukf_var_pitch_, cxt_.ukf_var_yaw_;
  filter_.set_Q(ukf::Q_discrete_white_noise_Xv(NUM_DER, 1 / orca::Model::CAMERA_FREQ, variances));

  // State transition function
  filter_.set_f_fn(
    [this](const double dt, const Eigen::VectorXd & u, Eigen::Ref<Eigen::VectorXd> x)
    {
      if (cxt_.predict_accel_) {
        // Assume 0 acceleration
        px_ax = 0;
        px_ay = 0;
        px_az = 0;
        px_aroll = 0;
        px_apitch = 0;
        px_ayaw = 0;

        if (cxt_.predict_accel_control_) {
          // Add acceleration due to control
          px_ax += u(0, 0);
          px_ay += u(1, 0);
          px_az += u(2, 0);
          px_ayaw += u(3, 0);
        }

        if (cxt_.predict_accel_drag_) {
          // Add acceleration due to drag
          // TODO(clyde): create & use AddLinkForce(drag_force, c_of_mass)
          //  and AddRelativeTorque(drag_torque)
          // Simple approximation:
          px_ax += cxt_.drag_accel_f(px_vx);      // TODO(clyde): f or x?
          px_ay += cxt_.drag_accel_s(px_vy);      // TODO(clyde): s or y?
          px_az += cxt_.drag_accel_z(px_vz);
          px_aroll += cxt_.drag_accel_yaw(px_vroll);
          px_apitch += cxt_.drag_accel_yaw(px_vpitch);
          px_ayaw += cxt_.drag_accel_yaw(px_vyaw);
        }

        if (cxt_.predict_accel_buoyancy_) {
          // Add acceleration due to gravity and buoyancy
          // TODO(clyde): create & use AddLinkForce(buoyancy_force, c_of_volume)
          // Simple approximation:
          px_roll = 0;
          px_pitch = 0;
          px_az -= cxt_.hover_accel_z();
        }
      }

      // Clamp acceleration
      px_ax = orca::clamp(px_ax, MAX_PREDICTED_ACCEL_XYZ);
      px_ay = orca::clamp(px_ay, MAX_PREDICTED_ACCEL_XYZ);
      px_az = orca::clamp(px_az, MAX_PREDICTED_ACCEL_XYZ);
      px_aroll = orca::clamp(px_aroll, MAX_PREDICTED_ACCEL_RPY);
      px_apitch = orca::clamp(px_apitch, MAX_PREDICTED_ACCEL_RPY);
      px_ayaw = orca::clamp(px_ayaw, MAX_PREDICTED_ACCEL_RPY);

      // Velocity, vx += ax * dt
      px_vx += px_ax * dt;
      px_vy += px_ay * dt;
      px_vz += px_az * dt;
      px_vroll += px_aroll * dt;
      px_vpitch += px_apitch * dt;
      px_vyaw += px_ayaw * dt;

      // Clamp velocity
      px_vx = orca::clamp(px_vx, MAX_PREDICTED_VELO_XYZ);
      px_vy = orca::clamp(px_vy, MAX_PREDICTED_VELO_XYZ);
      px_vz = orca::clamp(px_vz, MAX_PREDICTED_VELO_XYZ);
      px_vroll = orca::clamp(px_vroll, MAX_PREDICTED_VELO_RPY);
      px_vpitch = orca::clamp(px_vpitch, MAX_PREDICTED_VELO_RPY);
      px_vyaw = orca::clamp(px_vyaw, MAX_PREDICTED_VELO_RPY);

      // Position, x += vx * dt
      px_x += px_vx * dt;
      px_y += px_vy * dt;
      px_z += px_vz * dt;
      px_roll = orca::norm_angle(px_roll + px_vroll * dt);
      px_pitch = orca::norm_angle(px_pitch + px_vpitch * dt);
      px_yaw = orca::norm_angle(px_yaw + px_vyaw * dt);
    });

  // Custom residual and mean functions
  filter_.set_r_x_fn(six_state_residual);
  filter_.set_mean_x_fn(six_state_mean);
}

void PoseFilter6D::pose_from_filter(geometry_msgs::msg::Pose & filtered_pose)
{
  pose_from_px(filter_.x(), filtered_pose);
}

void PoseFilter6D::fp_from_filter(orca_msgs::msg::FiducialPose & filtered_fp)
{
  pose_from_filter(filtered_fp.pose.pose);
  // twist_from_px(filter_.x(), filtered_fp.twist.twist);
  flatten_6x6_covar(filter_.P(), filtered_fp.pose.covariance, 0);
  // flatten_6x6_covar(filter_.P(), filtered_fp.twist.covariance, 6);
}

Measurement PoseFilter6D::to_measurement(
  const orca_msgs::msg::Depth & depth,
  const mw::Observations & observations) const
{
  Measurement m;
  m.init_z(depth, observations,
    [](const Eigen::Ref<const Eigen::VectorXd> & x, Eigen::Ref<Eigen::VectorXd> z)
    {
      z(0) = px_z;
    });
  return m;
}

Measurement PoseFilter6D::to_measurement(
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose,
  const mw::Observations & observations) const
{
  Measurement m;
  m.init_6dof(pose, observations,
    [](const Eigen::Ref<const Eigen::VectorXd> & x, Eigen::Ref<Eigen::VectorXd> z)
    {
      z(0) = px_x;
      z(1) = px_y;
      z(2) = px_z;
      z(3) = px_roll;
      z(4) = px_pitch;
      z(5) = px_yaw;
    });
  return m;
}

}  // namespace orca_filter
