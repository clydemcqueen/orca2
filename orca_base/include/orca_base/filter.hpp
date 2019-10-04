#ifndef ORCA_BASE_FILTER_H
#define ORCA_BASE_FILTER_H

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ukf/ukf.hpp"

#include "orca_msgs/msg/depth.hpp"

#include "orca_base/filter_context.hpp"
#include "orca_base/geometry.hpp"
#include "orca_base/message_queue.hpp"

namespace orca_base
{

  class Filter
  {
    const rclcpp::Duration LAG{RCL_MS_TO_NS(50)};
    const rclcpp::Duration TOO_OLD{RCL_MS_TO_NS(250)};

    rclcpp::Logger logger_;

    // Messages queues
    MessageQueue<orca_msgs::msg::Depth> depth_q_;
    MessageQueue<geometry_msgs::msg::PoseWithCovarianceStamped> pose_q_;

    // Filter
    ukf::UnscentedKalmanFilter filter_;
    rclcpp::Time filter_time_{};

    void predict(const rclcpp::Time &stamp, const Acceleration &u_bar);

    void process_depth(const Acceleration &u_bar, nav_msgs::msg::Odometry &filtered_odom);

    void process_pose(const Acceleration &u_bar, nav_msgs::msg::Odometry &filtered_odom);

  public:

    explicit Filter(const rclcpp::Logger &logger, const FilterContext &cxt_);

    bool filter_valid()
    { return filter_.valid(); }

    void queue_depth(const orca_msgs::msg::Depth &depth);

    void queue_pose(const geometry_msgs::msg::PoseWithCovarianceStamped &pose);

    // Run the filter forward to time t, return true if the filter is valid
    bool process(const rclcpp::Time &t, const Acceleration &u_bar, nav_msgs::msg::Odometry &filtered_odom);
  };

} // namespace orca_base

#endif // ORCA_BASE_FILTER_H
