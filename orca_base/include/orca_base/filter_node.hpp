#ifndef ORCA_BASE_FILTER_NODE_HPP
#define ORCA_BASE_FILTER_NODE_HPP

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_msgs/msg/tf_message.hpp"

#include "orca_base/kf.hpp"

namespace filter_node {

class FilterNode : public rclcpp::Node
{
  // Parameters
  std::string map_frame_;
  std::string base_frame_;

  // Pose of base_frame in camera_frame
  tf2::Transform t_camera_base_;

  // Node state
  bool mission_;
  rclcpp::Time prev_stamp_;
  kf::KalmanFilter filter_;
  nav_msgs::msg::Path path_msg_;

  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr camera_pose_sub_;

  // Publications
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr filtered_odom_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;

  void camera_pose_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

public:

  explicit FilterNode();
  ~FilterNode() {}
};

} // namespace filter_node

#endif // ORCA_BASE_FILTER_NODE_HPP
