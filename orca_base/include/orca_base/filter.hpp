#ifndef ORCA_BASE_FILTER_H
#define ORCA_BASE_FILTER_H

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "orca_msgs/msg/depth.hpp"
#include "orca_base/base_context.hpp"
#include "ukf/ukf.hpp"

#include "geometry.hpp"

namespace orca_base
{
  class Filter
  {
    rclcpp::Logger logger_;

    ukf::UnscentedKalmanFilter filter_;
    rclcpp::Time filter_time_{};
    orca_msgs::msg::Depth depth_q_{};

    void predict(const Acceleration &u_bar, const rclcpp::Time &stamp);

    void update(const Eigen::MatrixXd &z, const Eigen::MatrixXd &R, const builtin_interfaces::msg::Time &stamp,
                nav_msgs::msg::Odometry &filtered_odom);

    void process_depth(const Acceleration &u_bar, const orca_msgs::msg::Depth &depth,
                       nav_msgs::msg::Odometry &filtered_odom);

    void process_odom(const Acceleration &u_bar, const nav_msgs::msg::Odometry &odom,
                      nav_msgs::msg::Odometry &filtered_odom);

  public:

    explicit Filter(const rclcpp::Logger &logger, const BaseContext &cxt_);

    bool filter_valid()
    { return filter_.valid(); }

    // Always succeeds
    void queue_depth(const orca_msgs::msg::Depth &depth);

    // Return true if we have good results in filtered_odom
    bool filter_odom(const Acceleration &u_bar, const nav_msgs::msg::Odometry &odom,
                     nav_msgs::msg::Odometry &filtered_odom);
  };

} // namespace orca_base

#endif // ORCA_BASE_FILTER_H
