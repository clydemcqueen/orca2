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

#include "orca_filter/pose_filter_node.hpp"

#include <memory>
#include <string>

#include "orca_filter/perf.hpp"
#include "orca_shared/mw/efforts.hpp"
#include "orca_shared/util.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

/***************************************************************************************************
 * PoseFilterNode runs one of three filters:
 *
 * PoseFilter6D filters all 6 DoF
 * PoseFilter4D filters x, y, z and yaw, and sets roll and pitch to 0
 * PoseFilter1D filters z, all other DoF are set to 0
 *
 * PoseFilterNode can be in one of two states:
 *
 * !good_pose:      There are no marker observations, or the markers are too far away to be useful
 *                   Use a PoseFilter1D at the barometer rate -- 20Hz
 *                   Output pose is (0, 0, filtered z, 0, 0, 0)
 *                   Output observations are copied from the last FiducialPoseStamped message
 *                   received, or no observations if the last FiducialPoseStamped message is stale
 *
 * good_pose:        Markers are close enough to generate a good pose
 *                   Use a PoseFilter6D or a PoseFilter4D at the camera rate -- 30Hz
 *                   FiducialPose observations are passed through unfiltered
 *                   Depth messages are fused, but don't result in published odometry
 */

namespace orca_filter
{

//=============================================================================
// FilterNode
//=============================================================================

constexpr int QUEUE_SIZE = 10;

PoseFilterNode::PoseFilterNode()
: Node{"pose_filter_node"}
{
  // Suppress IDE warnings
  (void) depth_sub_;
  (void) control_sub_;
  (void) fcam_sub_;

  // Create this before calling validate_parameters()
  filtered_odom_pub_ =
    create_publisher<orca_msgs::msg::FiducialPoseStamped>("filtered_fp", QUEUE_SIZE);

  // Get parameters, this will immediately call validate_parameters()
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), cxt_, n, t, d)
  CXT_MACRO_INIT_PARAMETERS(POSE_FILTER_NODE_ALL_PARAMS, validate_parameters)

  // Register parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(cxt_, n, t)
  CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), POSE_FILTER_NODE_ALL_PARAMS, validate_parameters)

  // Log parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_INFO, get_logger(), cxt_, n, t, d)
  POSE_FILTER_NODE_ALL_PARAMS

  // Check that all command line parameters are defined
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_CHECK_CMDLINE_PARAMETER(n, t, d)
  CXT_MACRO_CHECK_CMDLINE_PARAMETERS((*this), POSE_FILTER_NODE_ALL_PARAMS)

  // Parse URDF
  if (!parser_.parse()) {
    RCLCPP_ERROR(get_logger(), "can't parse URDF %s", orca_description::filename);
  }

  RCLCPP_INFO(get_logger(), "pose_filter_node ready");
}

void PoseFilterNode::validate_parameters()
{
  // Set up additional publications and subscriptions
  if (cxt_.filter_baro_) {
    depth_sub_ = create_subscription<orca_msgs::msg::Depth>(
      "depth", QUEUE_SIZE, [this](const orca_msgs::msg::Depth::SharedPtr msg) -> void
      {this->depth_cb_.call(msg);});
  } else {
    depth_sub_.reset();
  }

  if (cxt_.filter_fcam_) {
    fcam_sub_ = create_subscription<orca_msgs::msg::FiducialPoseStamped>(
      "fcam_fp", QUEUE_SIZE,
      [this](const orca_msgs::msg::FiducialPoseStamped::SharedPtr msg) -> void
      {this->fcam_cb_.call(msg);});
  } else {
    fcam_sub_.reset();
  }

  if (cxt_.predict_accel_ && cxt_.predict_accel_control_) {
    control_sub_ = create_subscription<orca_msgs::msg::Control>(
      "control", QUEUE_SIZE,
      [this](const orca_msgs::msg::Control::SharedPtr msg) -> void
        {this->control_cb_.call(msg);});
  } else {
    control_sub_.reset();
  }

  if (cxt_.publish_tf_) {
    tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf", QUEUE_SIZE);
  } else {
    tf_pub_.reset();
  }

  // Update timeouts
  open_water_timeout_ = rclcpp::Duration{RCL_MS_TO_NS(cxt_.timeout_open_water_ms_)};
  outlier_timeout_ = rclcpp::Duration{RCL_MS_TO_NS(cxt_.timeout_outlier_ms_)};

  // Nuke any existing filter
  filter_.reset();

  // Create a filter
  // TODO(clyde) would be nice if we passed in a pose
  create_filter(geometry_msgs::msg::Pose{});
}

// Create the appropriate filter, return true if we have a good pose
bool PoseFilterNode::create_filter(const geometry_msgs::msg::Pose & pose)
{
  auto good_pose = observations_.closest_distance() < cxt_.good_pose_dist_;

  auto expected_type = good_pose ?
    (cxt_.four_dof_ ? PoseFilterBase::Type::pose_4d : PoseFilterBase::Type::pose_6d) :
    PoseFilterBase::Type::pose_1d;

  if (!filter_ || filter_->type() != expected_type) {
    if (expected_type == PoseFilterBase::Type::pose_1d) {
      filter_ = std::make_shared<PoseFilter1D>(get_logger(), cxt_, filtered_odom_pub_, tf_pub_);
    } else if (expected_type == PoseFilterBase::Type::pose_4d) {
      filter_ = std::make_shared<PoseFilter4D>(get_logger(), cxt_, filtered_odom_pub_, tf_pub_);
    } else {
      filter_ = std::make_shared<PoseFilter6D>(get_logger(), cxt_, filtered_odom_pub_, tf_pub_);
    }

    // Always init!
    filter_->init(pose);
  }

  return good_pose;
}

// New depth reading
void PoseFilterNode::depth_callback(const orca_msgs::msg::Depth::SharedPtr msg, bool first)
{
  START_PERF()

  if (cxt_.filter_baro_) {
    if (!observations_.empty()) {
      rclcpp::Time stamp{msg->header.stamp};
      if (orca::valid_stamp(last_fp_stamp_) && stamp - last_fp_stamp_ > open_water_timeout_) {
        // Timeout... clear observations and create a 1d filter
        observations_.clear();
        geometry_msgs::msg::Pose pose;
        pose.position.z = msg->z;
        create_filter(pose);
      }
    }

    filter_->process_message(*msg, observations_, u_bar_);
  }

  STOP_PERF("depth_callback")
}

void PoseFilterNode::control_callback(const orca_msgs::msg::Control::SharedPtr msg, bool first)
{
  mw::Efforts efforts{msg->efforts};
  u_bar_ = efforts.acceleration(cxt_, estimated_yaw_);
}

void PoseFilterNode::fcam_callback(const orca_msgs::msg::FiducialPoseStamped::SharedPtr msg, bool first)
{
  START_PERF()

  if (cxt_.filter_fcam_) {
    process_pose(msg, parser_.t_fcam_base, "fcam_measurement");
  }

  STOP_PERF("fcam_callback")
}

void PoseFilterNode::process_pose(
  const orca_msgs::msg::FiducialPoseStamped::SharedPtr & msg,
  const tf2::Transform & t_sensor_base, const std::string & frame_id)
{
  last_fp_stamp_ = msg->header.stamp;

  // Save observations
  observations_ = mw::Observations{msg->fp.observations};

  // Make sure we have the appropriate filter, return if we don't have a good pose
  if (!create_filter(msg->fp.pose.pose)) {
    return;
  }

  // If we're receiving poses but not publishing odometry then re-initialize the filter
  if (orca::valid_stamp(last_fp_inlier_stamp_) &&
    last_fp_stamp_ - last_fp_inlier_stamp_ > outlier_timeout_)
  {
    RCLCPP_DEBUG(get_logger(), "init filter");
    filter_->init(msg->fp.pose.pose);
    last_fp_inlier_stamp_ = last_fp_stamp_;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped pose_stamped;
  pose_stamped.header = msg->header;
  pose_stamped.pose = msg->fp.pose;
  if (filter_->process_message(pose_stamped, observations_, u_bar_)) {
    last_fp_inlier_stamp_ = last_fp_stamp_;
  }
}

}  // namespace orca_filter

//=============================================================================
// Main
//=============================================================================

#ifdef RUN_PERF
void print_mean(std::string msg, int samples[])
{
  int sum = 0;
  for (int i = 0; i < NUM_SAMPLES; ++i) {
    sum += samples[i];
  }
  std::cout << msg << " ave " << sum / NUM_SAMPLES << " microseconds" << std::endl;
}
#endif

int main(int argc, char ** argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Init node
  auto node = std::make_shared<orca_filter::PoseFilterNode>();

  // Set logger level
  auto result =
    rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);
  (void) result;

  // Spin node
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
