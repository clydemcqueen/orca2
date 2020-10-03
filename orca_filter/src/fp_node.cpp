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

#include <memory>
#include <string>

#include "fiducial_vlam_msgs/msg/observations.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/exact_time.h"
#include "orca_description/parser.hpp"
#include "orca_shared/mw/fiducial_pose_stamped.hpp"
#include "orca_shared/monotonic.hpp"
#include "ros2_shared/context_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_msgs/msg/tf_message.hpp"

namespace orca_filter
{

//=============================================================================
// Parameter(s)
//=============================================================================

#define FP_NODE_ALL_PARAMS \
  CXT_MACRO_MEMBER(map_frame, std::string, "map")             /* Map frame  */ \
  CXT_MACRO_MEMBER(base_frame, std::string, "base_link")      /* Base frame  */ \
 \
  CXT_MACRO_MEMBER(publish_tf, bool, false)                   /* Publish t_map_base  */ \
 \
  CXT_MACRO_MEMBER(marker_length, double, 0.1778)             /* Marker length in meters  */ \
/* End of list */

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)

struct FPContext
{
  FP_NODE_ALL_PARAMS
};

//=============================================================================
// FPNode subscribes to PoseWithCovarianceStamped and fiducial_vlam::msg::Observations
// and publishes orca_msgs::msg::FiducialPose
//=============================================================================

constexpr int QUEUE_SIZE = 10;

class FPNode : public rclcpp::Node
{
  FPContext cxt_;                  // Parameter(s)

  // Parsed URDF
  orca_description::Parser parser_;

  // Message filter subscriptions
  message_filters::Subscriber<fiducial_vlam_msgs::msg::Observations> obs_sub_;
  message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped> pose_sub_;

  // Sync pose + observations
  using FiducialPolicy = message_filters::sync_policies::ExactTime<
    fiducial_vlam_msgs::msg::Observations,
    geometry_msgs::msg::PoseWithCovarianceStamped>;
  using FiducialSync = message_filters::Synchronizer<FiducialPolicy>;
  std::shared_ptr<FiducialSync> sync_;

  // Publications
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  rclcpp::Publisher<orca_msgs::msg::FiducialPoseStamped>::SharedPtr fp_pub_;

  /**
   * Get a synchronized set of messages from vloc: pose and marker observations
   *
   * Guard against invalid or duplicate timestamps
   * vloc poses are sensor_f_map -- transform to base_f_map
   *
   * @param obs_msg Marker observations
   * @param pose_msg Resulting pose from SolvePnP
   */
  void fiducial_callback(
    const fiducial_vlam_msgs::msg::Observations::ConstSharedPtr & obs_msg,
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & pose_msg)
  {
    // Keep track of the previous time
    static rclcpp::Time prev_time{0, 0, RCL_ROS_TIME};

    // The current time
    rclcpp::Time curr_time = obs_msg->header.stamp;

    // Skip first message
    if (!monotonic::valid(prev_time)) {
      prev_time = curr_time;
      return;
    }

    // Ignore invalid timestamps
    if (!monotonic::valid(curr_time)) {
      return;
    }

    // Ignore duplicates, or time going backwards
    // (Possible in simulation if we're running ROS time, see notes in orca_gazebo README)
    if (curr_time <= prev_time) {
      return;
    }

    // Convert pose to transform
    tf2::Transform t_map_sensor;
    tf2::fromMsg(pose_msg->pose.pose, t_map_sensor);

    // Multiply transforms to get t_map_base
    // This is tied to fcam... fix this to support multiple cameras
    tf2::Transform t_map_base = t_map_sensor * parser_.t_fcam_base;

    // Convert transform back to pose
    geometry_msgs::msg::PoseWithCovariance base_f_map;
    toMsg(t_map_base, base_f_map.pose);
    base_f_map.covariance = pose_msg->pose.covariance;  // TODO(clyde): rotate covariance

    // Publish tf
    if (cxt_.publish_tf_ && tf_pub_->get_subscription_count() > 0) {
      // Build transform message
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = curr_time;
      transform.header.frame_id = cxt_.map_frame_;
      transform.child_frame_id = cxt_.base_frame_;
      transform.transform = toMsg(t_map_base);

      // TF messages can have multiple transforms, we have just 1
      tf2_msgs::msg::TFMessage tf_message;
      tf_message.transforms.emplace_back(transform);

      tf_pub_->publish(tf_message);
    }

    geometry_msgs::msg::Pose cam_f_base;
    toMsg(parser_.t_base_fcam, cam_f_base);  // TODO(clyde): want parser.cam_f_base

    // Resulting fiducial pose
    mw::FiducialPoseStamped fp{cxt_.marker_length_, cam_f_base, *obs_msg, base_f_map};

    // Publish
    fp_pub_->publish(fp.msg());

    // Update prev_time
    prev_time = curr_time;
  }

  void fiducial_drop_callback(
    const fiducial_vlam_msgs::msg::Observations::ConstSharedPtr & obs_msg,
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & pose_msg)
  {
    if (obs_msg) {
      RCLCPP_DEBUG(get_logger(), "drop: have an observation but no odometry");
    } else {
      RCLCPP_DEBUG(get_logger(), "drop: have odometry but no observation");
    }
  }

  // Validate parameters
  void validate_parameters()
  {
    if (cxt_.publish_tf_) {
      tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf", QUEUE_SIZE);
    } else {
      tf_pub_.reset();
    }
  }

public:
  FPNode()
  : Node{"fp_node"}
  {
    // Get parameters, this will immediately call validate_parameters()
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), cxt_, n, t, d)
    CXT_MACRO_INIT_PARAMETERS(FP_NODE_ALL_PARAMS, validate_parameters)

    // Register parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(cxt_, n, t)
    CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), FP_NODE_ALL_PARAMS, validate_parameters)

    // Log parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_INFO, get_logger(), cxt_, n, t, d)
    FP_NODE_ALL_PARAMS

    // Message filter subscriptions
    // Default qos is reliable, queue=10
    obs_sub_.subscribe(this, "/fiducial_observations");
    pose_sub_.subscribe(this, "camera_pose");

    // Start sync
    using namespace std::placeholders;
    sync_.reset(new FiducialSync(FiducialPolicy(QUEUE_SIZE), obs_sub_, pose_sub_));
    sync_->registerCallback(std::bind(&FPNode::fiducial_callback, this, _1, _2));
    sync_->registerDropCallback(std::bind(&FPNode::fiducial_drop_callback, this, _1, _2));

    // Publication
    fp_pub_ = create_publisher<orca_msgs::msg::FiducialPoseStamped>("fp", QUEUE_SIZE);

    // Parse URDF
    if (!parser_.parse()) {
      RCLCPP_ERROR(get_logger(), "can't parse URDF %s", orca_description::filename);
    }

    RCLCPP_INFO(get_logger(), "fp_node ready");
  }

  ~FPNode() override = default;
};

}  // namespace orca_filter

//=============================================================================
// Main
//=============================================================================

int main(int argc, char ** argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Init node
  auto node = std::make_shared<orca_filter::FPNode>();

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
