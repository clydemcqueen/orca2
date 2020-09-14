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

#ifndef ORCA_FILTER__FILTER_NODE_HPP_
#define ORCA_FILTER__FILTER_NODE_HPP_

#include <memory>
#include <string>

#include "orca_description/parser.hpp"
#include "orca_filter/filter_context.hpp"
#include "orca_filter/pose_filter_base.hpp"
#include "orca_msgs/msg/control.hpp"
#include "orca_msgs/msg/depth.hpp"
#include "orca_msgs/msg/fiducial_pose_stamped.hpp"
#include "orca_shared/monotonic.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_msgs/msg/tf_message.hpp"

namespace orca_filter
{

class FilterNode : public rclcpp::Node
{
  // Parameters
  FilterContext cxt_;

  // Timeouts, set by parameters
  rclcpp::Duration open_water_timeout_{0};
  rclcpp::Duration outlier_timeout_{0};

  // Parsed URDF
  orca_description::Parser parser_;

  // Filter state
  bool good_pose_{false};
  std::shared_ptr<PoseFilterBase> filter_;
  rclcpp::Time last_fp_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_fp_inlier_stamp_{0, 0, RCL_ROS_TIME};
  mw::Observations observations_{};

  // Control state
  // double estimated_yaw_{};       // Yaw used to rotate thruster commands into the world frame
  // orca::Acceleration u_bar_{};   // Last control, used for filter predict step

  rclcpp::Publisher<orca_msgs::msg::FiducialPoseStamped>::SharedPtr filtered_odom_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;

  rclcpp::Subscription<orca_msgs::msg::Depth>::SharedPtr depth_sub_;
  rclcpp::Subscription<orca_msgs::msg::Control>::SharedPtr control_sub_;
  rclcpp::Subscription<orca_msgs::msg::FiducialPoseStamped>::SharedPtr fcam_sub_;

  // Validate parameters
  void validate_parameters();

  // Using observations_, set the state and create the appropriate filter
  void create_filter();

  // Callbacks
  void depth_callback(orca_msgs::msg::Depth::SharedPtr msg, bool first);

  void control_callback(orca_msgs::msg::Control::SharedPtr msg, bool first);

  void fcam_callback(orca_msgs::msg::FiducialPoseStamped::SharedPtr msg, bool first);

  // Callback wrappers
  monotonic::Monotonic<FilterNode *, const orca_msgs::msg::Depth::SharedPtr>
  depth_cb_{this, &FilterNode::depth_callback};
  monotonic::Monotonic<FilterNode *, const orca_msgs::msg::Control::SharedPtr>
  control_cb_{this, &FilterNode::control_callback};
  monotonic::Monotonic<FilterNode *, const orca_msgs::msg::FiducialPoseStamped::SharedPtr>
  fcam_cb_{this, &FilterNode::fcam_callback};

  // Process a camera pose
  void process_pose(
    const orca_msgs::msg::FiducialPoseStamped::SharedPtr & msg,
    const tf2::Transform & t_sensor_base, const std::string & frame_id);

public:
  FilterNode();

  ~FilterNode() override = default;
};

}  // namespace orca_filter

#endif  // ORCA_FILTER__FILTER_NODE_HPP_
