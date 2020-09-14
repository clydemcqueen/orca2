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

#ifndef ORCA_FILTER__POSE_FILTER_BASE_HPP_
#define ORCA_FILTER__POSE_FILTER_BASE_HPP_

#include <deque>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "orca_filter/filter_context.hpp"
#include "orca_msgs/msg/depth.hpp"
#include "orca_shared/mw/acceleration.hpp"
#include "orca_shared/mw/fiducial_pose_stamped.hpp"
#include "orca_shared/mw/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "ukf/ukf.hpp"

namespace orca_filter
{

//=============================================================================
// Constants
//=============================================================================

// Maximum predicted acceleration
// The AUV may be tossed around by waves or bump into something
constexpr double MAX_PREDICTED_ACCEL_XYZ = 100;
constexpr double MAX_PREDICTED_ACCEL_RPY = 100;

// Maximum predicted velocity in water
constexpr double MAX_PREDICTED_VELO_XYZ = 100;
constexpr double MAX_PREDICTED_VELO_RPY = 100;

//==================================================================
// Unscented residual and mean functions for PoseFilter6D 6dof state (x) and 6dof pose measurement (z)
//
// Because of the layout of the state and pose measurement matrices and the way that Eigen works
// these functions can serve double-duty.
//
// 6d x:       [x, y, z, r, p, y, ...]
// 6d pose z:  [x, y, z, r, p, y]
//
// The mean function needs to compute the mean of angles, which doesn't have a precise meaning.
// See https://en.wikipedia.org/wiki/Mean_of_circular_quantities for the method used here.
//
// There are similar residual and mean functions for PoseFilter4D 4dof state (x) and 4dof pose
// measurement (z)
//==================================================================

Eigen::VectorXd
six_state_residual(const Eigen::Ref<const Eigen::VectorXd> & x, const Eigen::VectorXd & mean);

Eigen::VectorXd six_state_mean(const Eigen::MatrixXd & sigma_points, const Eigen::RowVectorXd & Wm);

Eigen::VectorXd
four_state_residual(const Eigen::Ref<const Eigen::VectorXd> & x, const Eigen::VectorXd & mean);

Eigen::VectorXd
four_state_mean(const Eigen::MatrixXd & sigma_points, const Eigen::RowVectorXd & Wm);

//=============================================================================
// Utility for 6dof covariance matrices
//=============================================================================

void flatten_6x6_covar(const Eigen::MatrixXd & m, std::array<double, 36> & covar, int offset);

//=============================================================================
// Measurements
//=============================================================================

struct Measurement
{
  enum class Type
  {
    depth, four, six
  };

  Type type_;
  rclcpp::Time stamp_;
  mw::Observations observations_;

  Eigen::VectorXd z_;
  Eigen::MatrixXd R_;
  ukf::MeasurementFn h_fn_;
  ukf::ResidualFn r_z_fn_;
  ukf::UnscentedMeanFn mean_z_fn_;

  // Must be default constructable to be used in a priority queue
  Measurement() = default;

  // 1dof z measurement from a depth message
  void init_z(
    const orca_msgs::msg::Depth & depth, const mw::Observations & observations,
    ukf::MeasurementFn h_fn);

  // 4dof measurement from a pose message
  void init_4dof(
    const geometry_msgs::msg::PoseWithCovarianceStamped & pose,
    const mw::Observations & observations, ukf::MeasurementFn h_fn);

  // 6dof measurement from a pose message
  void init_6dof(
    const geometry_msgs::msg::PoseWithCovarianceStamped & pose,
    const mw::Observations & observations, ukf::MeasurementFn h_fn);

  // Sort by time
  bool operator()(const Measurement & a, const Measurement & b)
  {
    return a.stamp_ > b.stamp_;
  }

  // Type name
  std::string name() const;
};

//=============================================================================
// Filter state
//=============================================================================

struct State
{
  rclcpp::Time stamp_;
  Eigen::VectorXd x_;
  Eigen::MatrixXd P_;

  // Must be default constructable
  State() = default;

  State(const rclcpp::Time & stamp, Eigen::VectorXd x, Eigen::MatrixXd P)
  : stamp_{stamp}, x_{std::move(x)}, P_{std::move(P)} {}
};

//=============================================================================
// Filter base
//=============================================================================

class PoseFilterBase
{
public:
  enum class Type
  {
    depth, four, pose
  };

private:
  const rclcpp::Duration HISTORY_LENGTH{RCL_S_TO_NS(1)};

  Type type_;                                 // Depth filter vs four filter vs pose filter
  int state_dim_;                             // Number of dimensions, 1 for depth filter, etc.
  rclcpp::Time filter_time_;                  // Current time of filter
  rclcpp::Time odom_time_;                    // Timestamp of last publish odom message

  rclcpp::Publisher<orca_msgs::msg::FiducialPoseStamped>::SharedPtr filtered_odom_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;

  // Measurement priority queue, sorted from oldest to newest
  std::priority_queue<Measurement, std::vector<Measurement>, Measurement> measurement_q_;

  // State history, ordered from oldest to newest
  std::deque<State> state_history_;

  // Measurement history, ordered from oldest to newest
  std::deque<Measurement> measurement_history_;

  /**
   * Call filter_->predict
   *
   * @param stamp Predict state at this timestamp
   * @param u_bar Control input
   */
  void predict(const rclcpp::Time & stamp, const mw::Acceleration & u_bar);

  /**
   * Process all messages in the queue and publish odometry as necessary
   *
   * @param stamp Timestamp of most recent message, used to trim state and measurement history
   * @return True if at least one measurement was an inlier and the filter is good
   */
  bool process_measurements(const rclcpp::Time & stamp);

  /**
   * Rewind the filter to a previous state
   *
   * @param stamp Rewind to this time
   * @return Return true if rewind was successful (i.e., history went back that far)
   */
  bool rewind(const rclcpp::Time & stamp);

  // Publish odometry
  void publish_odom(const Measurement & measurement);

protected:
  rclcpp::Logger logger_;
  const FilterContext & cxt_;

  ukf::UnscentedKalmanFilter filter_;

  // Reset the filter with an Eigen vector
  void reset(const Eigen::VectorXd & x);

  /**
   * Generate an odometry message from the current filter state
   *
   * @param filtered_odom Output: odometry message
   */
  virtual void odom_from_filter(orca_msgs::msg::FiducialPose & filtered_odom) = 0;

  /**
   * Convert a Depth message to a Measurement
   *
   * @param depth Incoming message
   * @return Measurement
   */
  virtual Measurement to_measurement(
    const orca_msgs::msg::Depth & depth,
    const mw::Observations & observations) const = 0;

  /**
   * Convert PoseWithCovarianceStamped message to a Measurement
   *
   * @param pose Incoming message
   * @return Measurement
   */
  virtual Measurement to_measurement(
    const geometry_msgs::msg::PoseWithCovarianceStamped & pose,
    const mw::Observations & observations) const = 0;

public:
  explicit PoseFilterBase(
    Type type,
    const rclcpp::Logger & logger,
    const FilterContext & cxt,
    rclcpp::Publisher<orca_msgs::msg::FiducialPoseStamped>::SharedPtr filtered_odom_pub,
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub,
    int state_dim);

  /**
   * Reset the filter with the default pose (0, 0, 0) and high uncertainty
   */
  void reset();

  /**
   * Reset the filter with an initial pose and high uncertainty
   *
   * Problem: if the first pose is an outlier the filter may settle on a bad solution and reject
   * all other poses. In theory this can be prevented by setting the uncertainty correctly. In
   * practice calculating uncertainty can be difficult, and the uncertainty might not be Gaussian.
   * For Orca2 the poses are generated by OpenCV ArUco detection and SolvePnP, and the uncertainty
   * is decidedly non-Gaussian.
   *
   * Workaround: initialize the filter with a reasonable estimate -- perhaps from the motion
   * planner.
   *
   * @param pose The initial pose
   */
  virtual void reset(const geometry_msgs::msg::Pose & pose) = 0;

  // Is the filter valid?
  bool filter_valid() {return filter_.valid();}

  /**
   * Send a message to the filter, and publish updated odometry
   *
   * @tparam T Type of message to process, each type requires a to_measurement(T) function
   * @param msg Message to process
   * @param observations Fiducial observations... these will be passed through (unfiltered) to the
   * published odometry
   * @return True if there was at least one inlier and the filter is good
   */
  template<typename T>
  bool process_message(const T & msg, const mw::Observations & observations)
  {
    rclcpp::Time stamp{msg.header.stamp};

    if (stamp < filter_time_ && !rewind(stamp)) {
      // This message is out of order, and we can't rewind history
      return false;
    }

    // Add this message to the priority queue
    measurement_q_.push(to_measurement(msg, observations));

    // Process one or more measurements
    return process_measurements(stamp);
  }

  Type type() {return type_;}

  std::string name();
};

//=============================================================================
// Filter only z (depth)
//=============================================================================

class PoseFilter1D : public PoseFilterBase
{
  void odom_from_filter(orca_msgs::msg::FiducialPose & filtered_odom) override;

  Measurement to_measurement(
    const orca_msgs::msg::Depth & depth,
    const mw::Observations & observations) const override;

  Measurement to_measurement(
    const geometry_msgs::msg::PoseWithCovarianceStamped & pose,
    const mw::Observations & observations) const override;

public:
  explicit PoseFilter1D(
    const rclcpp::Logger & logger,
    const FilterContext & cxt,
    rclcpp::Publisher<orca_msgs::msg::FiducialPoseStamped>::SharedPtr filtered_odom_pub,
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub);

  // Reset the filter with a pose
  void reset(const geometry_msgs::msg::Pose & pose) override;
};

//=============================================================================
// Filter 4 DoF, assume roll and pitch are always 0
//=============================================================================

class PoseFilter4D : public PoseFilterBase
{
  void odom_from_filter(orca_msgs::msg::FiducialPose & filtered_odom) override;

  Measurement to_measurement(
    const orca_msgs::msg::Depth & depth,
    const mw::Observations & observations) const override;

  Measurement to_measurement(
    const geometry_msgs::msg::PoseWithCovarianceStamped & pose,
    const mw::Observations & observations) const override;

public:
  explicit PoseFilter4D(
    const rclcpp::Logger & logger,
    const FilterContext & cxt,
    rclcpp::Publisher<orca_msgs::msg::FiducialPoseStamped>::SharedPtr filtered_odom_pub,
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub);

  // Reset the filter with a pose
  void reset(const geometry_msgs::msg::Pose & pose) override;
};

//=============================================================================
// Filter all 6 DoF
//=============================================================================

class PoseFilter6D : public PoseFilterBase
{
  void odom_from_filter(orca_msgs::msg::FiducialPose & filtered_odom) override;

public:
  explicit PoseFilter6D(
    const rclcpp::Logger & logger,
    const FilterContext & cxt,
    rclcpp::Publisher<orca_msgs::msg::FiducialPoseStamped>::SharedPtr filtered_odom_pub,
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub);

  // Reset the filter with a pose
  void reset(const geometry_msgs::msg::Pose & pose) override;

  Measurement to_measurement(
    const orca_msgs::msg::Depth & depth,
    const mw::Observations & observations) const override;

  Measurement to_measurement(
    const geometry_msgs::msg::PoseWithCovarianceStamped & pose,
    const mw::Observations & observations) const override;
};

}  // namespace orca_filter

#endif  // ORCA_FILTER__POSE_FILTER_BASE_HPP_
