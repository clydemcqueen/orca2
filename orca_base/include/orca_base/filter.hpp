#ifndef ORCA_BASE_FILTER_H
#define ORCA_BASE_FILTER_H

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "orca_msgs/msg/barometer.hpp"

#include "ukf/ukf.hpp"

namespace orca_base
{
  class Filter
  {
    ukf::UnscentedKalmanFilter filter_;

  public:
    explicit Filter();

    void filter_odom(nav_msgs::msg::Odometry::SharedPtr msg,
                     geometry_msgs::msg::PoseWithCovarianceStamped &pose);

    void filter_baro(orca_msgs::msg::Barometer::SharedPtr msg,
                     geometry_msgs::msg::PoseWithCovarianceStamped &pose);
  };

} // namespace orca_base

#endif // ORCA_BASE_FILTER_H
