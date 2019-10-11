#ifndef ORCA_BASE_FILTER_H
#define ORCA_BASE_FILTER_H

#include <queue>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ukf/ukf.hpp"

#include "orca_msgs/msg/depth.hpp"

#include "orca_base/filter_context.hpp"
#include "orca_base/geometry.hpp"

namespace orca_base
{

  struct Measurement
  {
    rclcpp::Time stamp_;
    Eigen::VectorXd z_;
    Eigen::MatrixXd R_;
    ukf::MeasurementFn h_fn_;
    ukf::ResidualFn r_z_fn_;
    ukf::UnscentedMeanFn mean_z_fn_;

    // Must be default constructible
    Measurement() = default;

    explicit Measurement(const orca_msgs::msg::Depth &depth);

    explicit Measurement(const geometry_msgs::msg::PoseWithCovarianceStamped &pose);

    // Sort by time
    bool operator()(const Measurement &a, const Measurement &b)
    {
      return a.stamp_ > b.stamp_;
    }
  };

  class Filter
  {
    const rclcpp::Duration LAG{RCL_MS_TO_NS(50)};
    const rclcpp::Duration TOO_OLD{RCL_MS_TO_NS(250)};

    rclcpp::Logger logger_;
    const FilterContext &cxt_;

    // Measurement queue
    std::priority_queue<Measurement, std::vector<Measurement>, Measurement> q_;

    // Filter
    ukf::UnscentedKalmanFilter filter_;
    rclcpp::Time filter_time_{0, 0, RCL_ROS_TIME};

    void predict(const rclcpp::Time &stamp, const Acceleration &u_bar);

  public:

    explicit Filter(const rclcpp::Logger &logger, const FilterContext &cxt_);

    void reset();

    bool filter_valid()
    { return filter_.valid(); }

    void queue_depth(const orca_msgs::msg::Depth &depth);

    void queue_pose(const geometry_msgs::msg::PoseWithCovarianceStamped &pose);

    // Run the filter, return true if there's a odometry message to publish
    bool process(const rclcpp::Time &t, const Acceleration &u_bar, nav_msgs::msg::Odometry &filtered_odom);
  };

} // namespace orca_base

#endif // ORCA_BASE_FILTER_H
