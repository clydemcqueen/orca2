#ifndef ORCA_BASE_FILTER_H
#define ORCA_BASE_FILTER_H

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "orca_msgs/msg/barometer.hpp"
#include "orca_base/base_context.hpp"
#include "ukf/ukf.hpp"

#include "geometry.hpp"

namespace orca_base
{
  class Filter
  {
    ukf::UnscentedKalmanFilter filter_;
    rclcpp::Time filter_time_{};
    orca_msgs::msg::Barometer baro_q_{};

    double dt(const rclcpp::Time &stamp);

    void process_baro(const Acceleration &u_bar, const orca_msgs::msg::Barometer &baro,
                      nav_msgs::msg::Odometry &filtered_odom);

    void process_odom(const Acceleration &u_bar, const nav_msgs::msg::Odometry &odom,
                      nav_msgs::msg::Odometry &filtered_odom);

  public:

    explicit Filter(const BaseContext &cxt_);

    bool filter_valid()
    { return filter_.valid(); }

    // Always succeeds
    void queue_baro(rclcpp::Logger logger, const orca_msgs::msg::Barometer &baro);

    // Return true if we have good results in filtered_odom
    bool filter_odom(rclcpp::Logger logger, const Acceleration &u_bar, const nav_msgs::msg::Odometry &odom,
                     nav_msgs::msg::Odometry &filtered_odom);
  };

} // namespace orca_base

#endif // ORCA_BASE_FILTER_H
