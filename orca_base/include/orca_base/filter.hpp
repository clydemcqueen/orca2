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

  public:
    explicit Filter(const BaseContext &cxt_);

    // Return true if filter is in a valid state
    bool filter_odom(double dt, const Acceleration &u_bar,
                     const nav_msgs::msg::Odometry &fiducial_odom,
                     nav_msgs::msg::Odometry &filtered_odom);
  };

} // namespace orca_base

#endif // ORCA_BASE_FILTER_H
